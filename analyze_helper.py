import math

R = 6371.0 

def distance(lat1,lat2,lon1,lon2):

    R = 6371.0 
    lat = [lat1,lat2]
    lon = [lon1,lon2]

    for i in range (2):
        lat[i] = math.radians(lat[i])
        lon[i] = math.radians(lon[i])

    dlat = lat[1] - lat[0]
    dlon = lon[1] - lon[0]

    a = (math.sin(dlat/2))**2 + math.cos(lat[0]) * math.cos(lat[1]) * math.sin(dlon/2)**2
    c= 2*math.atan2(math.sqrt(a),math.sqrt(1-a))
    return R * c


if __name__ == "__main__":
    print(distance(0,0,0,0))
