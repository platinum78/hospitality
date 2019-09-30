# Task type: 0 == idle, 1 == delivery, 2 == cleanup

int32 task_id
int32 task_type
float64 request_timestamp
float64 desired_finish_time
float64 allowable_delay
int32 priority_level
int32 start_place_id
int32 dest_place_id
string client