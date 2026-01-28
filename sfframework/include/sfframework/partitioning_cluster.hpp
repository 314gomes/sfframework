#include <any>
#include <map>
#include <vector>
#include <string>

struct PartitioningCluster {
    // Geometry: Indices pointing to the original cloud
    std::vector<size_t> indices;

    // Metadata: Flexible storage 
    std::map<std::string, std::any> attributes;
};