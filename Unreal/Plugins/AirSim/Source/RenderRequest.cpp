#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TaskGraphInterfaces.h"
#include "ImageUtils.h"

#include <Python.h>
#include "controllers/Settings.hpp"

static bool python_setup = true; //false;
static bool python_works = false; // true;
static PyObject * pModule;
static PyObject *pNoiseFunc;
const std::string module_name = "camera_noise";

RenderRequest::RenderRequest(bool use_safe_method)
{
    data = std::make_shared<RenderRequestInfo>();
    data->use_safe_method = use_safe_method;

	if (!python_setup) {
		python_setup = true;

		auto& settings = Settings::singleton();
		settings.initializePython();

		SetupPythonNoise();
	}
}
RenderRequest::~RenderRequest()
{
    data->render_target = nullptr;
    data = nullptr;
}

// read pixels from render target using render thread, then compress the result into PNG
// argument on the thread that calls this method.
void RenderRequest::getScreenshot(UTextureRenderTarget2D* renderTarget, TArray<uint8>& image_data_uint8, 
    TArray<float>& image_data_float, bool pixels_as_float, bool compress, int& width, int& height, uint64_t& timestamp, bool noisy, bool dead)
{
	auto end = std::chrono::system_clock::now();
	auto start_noise = std::chrono::system_clock::now();
	auto end_noise = std::chrono::system_clock::now();
	auto start = std::chrono::system_clock::now();

    data->render_target = renderTarget;
    if (!pixels_as_float)
        data->bmp.Reset();
    else
        data->bmp_float.Reset();
    data->pixels_as_float = pixels_as_float;
    data->compress = compress;

    //make sure we are not on the rendering thread
    CheckNotBlockedOnRenderThread();

    if (data->use_safe_method) {
        //TODO: below doesn't work right now because it must be running in game thread
        FIntPoint size;
        if (!data->pixels_as_float) {
            //below is documented method but more expensive because it forces flush
            FTextureRenderTargetResource* rt_resource = data->render_target->GameThread_GetRenderTargetResource();
            auto flags = setupRenderResource(rt_resource, data.get(), size);
            rt_resource->ReadPixels(data->bmp, flags);
        }
        else {
            FTextureRenderTargetResource* rt_resource = data->render_target->GetRenderTargetResource();
            setupRenderResource(rt_resource, data.get(), size);
            rt_resource->ReadFloat16Pixels(data->bmp_float);
        }
    }
    else {
        //wait for render thread to pick up our task

        // Queue up the task of rendering the scene in the render thread
        TGraphTask<RenderRequest>::CreateTask().ConstructAndDispatchWhenReady(*this);

        // wait for this task to complete
        if (!data->signal.waitFor(5)) {
            throw std::runtime_error("timeout waiting for screenshot");
        }
    }
    
    width = data->width;
    height = data->height;
	timestamp = data->timestamp__;

    if (!pixels_as_float) {
        if (data->width != 0 && data->height != 0) {
			if (dead) {
				static std::random_device rd{};
				static std::mt19937 gen{ rd() };

				for (auto& item : data->bmp) {
					std::uniform_int_distribution<int> dist(0, 256);

					item.R = dist(gen);
					item.G = dist(gen);
					item.B = dist(gen);
					item.A = dist(gen);
				}
			}
			else if (noisy) {
				// AddPythonNoise(data->bmp);

				start_noise = std::chrono::system_clock::now();

				static std::random_device rd{};
				static std::mt19937 gen{ rd() };

				for (auto& item : data->bmp) {
					// divide by 10 to get meters
					float depth = ((float) item.R);

					float real_depth = depth / 10.0f;
					float noise = 0.0f;
					float sd;
					if (real_depth < 6.0f) {
						sd = real_depth * 0.0035 - 0.0023;
					}
					else {
						sd = real_depth * 0.0657 - 0.4193;
					}
					sd = (sd < 0.0f) ? 0.0f : sd;
					std::normal_distribution<float> d(0.0f, sd);
					real_depth += d(gen);

					depth = real_depth * 10.0f;
					// clamp depth to uint8 range before conversion
					depth = depth < 0.0f ? 0.0f :
						depth > 255.0f ? 255.0f : depth;
					item.R = (uint8) depth;
				}

				end_noise = std::chrono::system_clock::now();
			}

            if (data->compress)
                FImageUtils::CompressImageArray(data->width, data->height, data->bmp, image_data_uint8);
            else {
                for (const auto& item : data->bmp) {
                    image_data_uint8.Add(item.R);
                    image_data_uint8.Add(item.G);
                    image_data_uint8.Add(item.B);
                    image_data_uint8.Add(item.A);
                }
            }
        }
    }
    else {
        for (const auto& item : data->bmp_float) {
            float fval = item.R.GetFloat();
            image_data_float.Add(fval);
        }
    }

	end = std::chrono::system_clock::now();

	volatile auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	volatile auto noise_diff = std::chrono::duration_cast<std::chrono::microseconds>(end_noise - start_noise).count();
	++diff;
	++noise_diff;
	--diff;
	--noise_diff;
	return;
}

FReadSurfaceDataFlags RenderRequest::setupRenderResource(FTextureRenderTargetResource* rt_resource, RenderRequestInfo* data, FIntPoint& size)
{
    size = rt_resource->GetSizeXY();
    data->width = size.X;
    data->height = size.Y;
    FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
    flags.SetLinearToGamma(false);

    return flags;
}

void RenderRequest::ExecuteTask()
{
    if (data != nullptr)
    {
        FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
        auto rt_resource = data->render_target->GetRenderTargetResource();
		//uint64 timestamp;
		if (rt_resource != nullptr) {
            const FTexture2DRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
            FIntPoint size;
            auto flags = setupRenderResource(rt_resource, data.get(), size);

            //should we be using ENQUEUE_UNIQUE_RENDER_COMMAND_ONEPARAMETER which was in original commit by @saihv
            //https://github.com/Microsoft/AirSim/pull/162/commits/63e80c43812300a8570b04ed42714a3f6949e63f#diff-56b790f9394f7ca1949ddbb320d8456fR64
			//uint64 blah = msr::airlib::ClockFactory::get()->nowNanos();
			
			if (!data->pixels_as_float) {
                //below is undocumented method that avoids flushing, but it seems to segfault every 2000 or so calls
				uint64 before_time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
                RHICmdList.ReadSurfaceData(
                    rhi_texture,
                    FIntRect(0, 0, size.X, size.Y),
                    data->bmp,
                    flags);
				// uint64 after_time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
				data->timestamp__ = before_time_stamp;
			}
            else {
				uint64 before_time_stamp  = msr::airlib::ClockFactory::get()->nowNanos();
				RHICmdList.ReadSurfaceFloatData(
                    rhi_texture,
                    FIntRect(0, 0, size.X, size.Y),
                    data->bmp_float,
                    CubeFace_PosX, 0, 0
				);
          
				// uint64 after_time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
				
				//if (after_time_stamp - before_time_stamp) > 
				// data->timestamp__ = (before_time_stamp+after_time_stamp)/2;
				data->timestamp__ = before_time_stamp;

			}
        }
		//data->timestamp__ = timestamp;
        data->signal.signal();
    }
}

void RenderRequest::SetupPythonNoise() {
	PyObject *pName;
	pName = PyUnicode_FromString(module_name.c_str());

	pModule = PyImport_Import(pName);

	if (pModule != NULL) {
		pNoiseFunc = PyObject_GetAttrString(pModule, "depth_noise");
		if (!(pNoiseFunc && PyCallable_Check(pNoiseFunc))) {
			//can't do it
			python_works = false;
		}
	}
	else {
		//can't do it
		python_works = false;
		PyErr_Print();
	}
}

void RenderRequest::AddPythonNoise(TArray<FColor>& bmp)
{
	for (auto& item : data->bmp) {
		PyObject * pArgs = PyTuple_New(1); // adding noise to x, y, z acceleration

		PyObject *pValue0 = PyFloat_FromDouble(double(item.R) / 10.0);
		PyTuple_SetItem(pArgs, 0, pValue0);
		
		// Returns tuple of three ordered values
		PyObject * pValue = PyObject_CallObject(pNoiseFunc, pArgs);
		if (pValue != NULL) {
			item.R = (uint8_t)(PyFloat_AsDouble(PyTuple_GetItem(pValue, 0)) * 10.0);
			Py_DECREF(pValue);
		}

		Py_DECREF(pArgs);
		Py_DECREF(pValue0);
	}
}
