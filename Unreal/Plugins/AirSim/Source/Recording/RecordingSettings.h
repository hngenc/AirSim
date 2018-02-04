#pragma once

struct RecordingSettings {
    bool record_on_move = false;
    float record_interval = 0.05f;
	bool record_upon_start = false;
	bool use_time_stamp_for_recording = true;
};