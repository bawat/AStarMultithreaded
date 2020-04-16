#include <string>

using namespace std;
/*
 * Used to create and return GML DSMaps.
 */
class DSMap {
	private:
		using ds_map_ID = int;
		using async_event_ID = int;

		typedef void(*GMLAsyncFunctionPointer) (ds_map_ID, int);
		typedef int(*GMLCreateFunctionPointer) (int, ...);
		typedef bool(*GMLMapAddDoubleFunctionPointer) (ds_map_ID, char*, double);
		typedef bool(*GMLMapAddStringFunctionPointer) (ds_map_ID, char*, char*);

		inline static GMLAsyncFunctionPointer gml_event_perform_async_internal;
		inline static GMLCreateFunctionPointer gml_ds_map_create_internal;
		inline static GMLMapAddDoubleFunctionPointer gml_ds_map_add_double_internal;
		inline static GMLMapAddStringFunctionPointer gml_ds_map_add_string_internal;
	public:
		static void registerCallbacks(char* gml_event_perform_async, char* gml_ds_map_create, char* gml_ds_map_add_double, char* gml_ds_map_add_string){
			gml_event_perform_async_internal = (void(*) (ds_map_ID, async_event_ID)) gml_event_perform_async;
			gml_ds_map_create_internal = (int(*) (int, ...)) gml_ds_map_create;
			gml_ds_map_add_double_internal = (bool(*) (ds_map_ID, char*, double)) gml_ds_map_add_double;
			gml_ds_map_add_string_internal = (bool(*) (ds_map_ID, char*, char*)) gml_ds_map_add_string;
		}

	private:
		ds_map_ID mapID;
	public:
		DSMap(){
			mapID = gml_ds_map_create_internal(0);
		}
		DSMap& addDouble(string key, double value){
			gml_ds_map_add_double_internal(mapID, (char*)key.c_str(), value);
			return *this;
		}
		DSMap& addString(string key, string value){
			gml_ds_map_add_string_internal(mapID, (char*)key.c_str(), (char*)value.c_str());
			return *this;
		}
		void sendToGMS2(){
			const int EVENT_OTHER_SOCIAL (70);
			gml_event_perform_async_internal(mapID, EVENT_OTHER_SOCIAL);
		}
		operator double() const { return mapID; }
};
