#pragma once
#include <vector>
#include <string>




struct KNN {
    int id;
    std::string poi;
    double lat, lon;
    int k;
    std::string metric;

    KNN( int id_, std::string poi_, double lat_, double lon_, int k_, std::string metric_) : id(id_) , poi(poi_) , lat(lat_) , lon(lon_),k(k_) , metric(metric_) {}
};
