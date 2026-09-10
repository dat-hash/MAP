import folium
import helper

m = folium.Map(location = [0 , 0], zoom_start= 15, tiles = "OpenStreetMap")
locations = [{"name": "start/stop", "lat": 10.721328051904697 , "lon": 10.74298280483535, "color": "red"}, {"name": "check point 1", "lat": 10.722958031840404 , "lon": 10.74300895564969, "color": "blue"}, {"name": "check point 2", "lat": 10.723140748372106 , "lon": 10.74131425593039, "color": "blue"}, {"name": "check point 3", "lat": 10.721411416603582 , "lon": 10.74120360354819, "color": "blue"}]

lon=[]
lat=[]
total_distance = 0

for point in locations:
    folium.Marker(location=[point["lat"],point["lon"]], popup= point["name"], icon = folium.Icon(color = point["color"], icon = "info-sign")).add_to(m)
    lon += [point["lon"]]
    lat += [point["lat"]]

for i in range(len(lat)):
    total_distance += helper.distance(lat[i],lat[(i+1)%len(lat)],lon[i],lon[(i+1)%len(lon)])

m.save("spot_check.html")
print ("map saved 'click spot_check.html'")
print(f"total distance: {total_distance:.2f}km")
