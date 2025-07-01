# Result for all knowledge requests (Get,Upd,Add)
# If the service was too busy to respond with valid data
# Error message, if result_type is not SUCCESS
string error
# Result types
int32 ERROR_ID_NOT_FOUND=1
int32 ERROR_SQL=2
int32 ERROR_DBAPI=3
int32 ERROR_BUSY = 4
int32 SUCCESS = 0
# Enum-like type of a result. One of the constants defined above.
int32 result_type
