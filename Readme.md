Declare as properties (Nontunable) customMap and hasCustomMap


Line 171 change:

obj.map = internal.createMapFromName(obj.mapName);            

for:

if obj.hasCustomMap
	obj.map = obj.customMap;
else
	obj.map = internal.createMapFromName(obj.mapName);
end