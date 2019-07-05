var R = 6371;

var lon1 = getprop("/position/longitude-deg");
var lat1 = getprop("/position/latitude-deg");

var lon2 = getprop("/position/longitude-deg");
var lat2 = getprop("/position/latitude-deg");

var lon_rad1 = (math.pi/180)*lon;
var lat_rad1 = (math.pi/180)*lat;

var lon_rad2 = (math.pi/180)*lon;
var lat_rad2 = (math.pi/180)*lat;

# calculate distance
var delta_theta = (lat_rad2-lat_rad1);
var delta_alpha = (lon_rad2-lon_rad1);

var a = math.sin(delta_theta/2) * math.sin(delta_theta/2) + math.cos(lon_rad1) * math.cos(lon_rad2) + math.sin(delta_alpha/2) * math.sin(delta_alpha/2);

var c = math.atan2(math.sqrt(a), math.sqrt(1-a));

var d = R * c;

print(d);

# calculate bearing
var x = math.sin(lon_rad2-lon_rad1) * math.cos(lat_rad2);
var y = math.cos(lat_rad1) * math.cos(lat_rad2) * math.sin(lat_rad1) * math.cos(lat_rad2 - lat_rad1);
var bearing = math.atan2(y,x)*(180/math.pi);

print(bearing);