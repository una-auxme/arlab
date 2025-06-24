# Result for all knowledge requests (Get,Upd,Add)
# If the service request was successful and the returned data is ok/valid
bool is_ok
# If the service was too busy to respond with valid data
# If is_busy is set, is_ok is always false
bool is_busy
# Error message, if is_ok is False
string error
