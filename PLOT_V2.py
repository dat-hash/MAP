import pandas as pd
import folium


print('program start')

df_gnss = pd.read_csv('PATH0001.CSV')
df_imu = pd.read_csv('IMU0001.CSV')

lon = df_gnss.iloc[1::10,20]
lat = df_gnss.iloc[1::10,19]

gyro_x = df_gnss
gyro_y = df_gnss
gyro_z = df_gnss

accel_x = df_gnss
accel_x = df_gnss
accel_x = df_gnss

print(lat)
print (lon)

map_center = [lat.mean(),lon.mean()]

m = folium.Map(map_center, zoom_start= 20, tiles = None)

folium.TileLayer(
    tiles="https://{s}.tile.openstreetmap.fr/osmfr/{z}/{x}/{y}.png",
    attr="&copy; OpenStreetMap France | &copy; OpenStreetMap contributors",
    name="OSM France"
).add_to(m)

for lt, ln in zip(lat, lon):
    folium.Marker(location=[lt,ln], icon = folium.Icon(color = "blue", icon = "info-sign")).add_to(m)

m.save("spot_checkv2.html")
print ("map saved 'click spot_checkv2.html'")
