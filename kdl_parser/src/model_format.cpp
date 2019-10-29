#include <kdl_parser/model_format.hpp>
#include <stdexcept>
#include <fstream>

using namespace kdl_parser;
using namespace std;

const char* kdl_parser::formatNameFromID(int type)
{
    switch(type)
    {
        case MODEL_UNKNOWN: return "unknown";
        case MODEL_AUTO: return "autodetected";
        case MODEL_URDF: return "URDF";
        case MODEL_SDF: return "SDF";
        default:
            throw std::invalid_argument("invalid model type ID given");
    }
}

MODEL_FORMAT kdl_parser::guessFormatFromFilename(const std::string& file)
{
    if (file.substr(file.size() - 4) == ".sdf")
        return MODEL_SDF;
    if (file.substr(file.size() - 5) == ".urdf")
        return MODEL_URDF;
    if (file.substr(file.size() - 6) == ".world")
        return MODEL_SDF;
    fprintf(stderr, "cannot guess robot model format for %s.\n", file.c_str());
    return MODEL_UNKNOWN;
}

MODEL_FORMAT kdl_parser::guessFormatFromString(const std::string& str)
{
    TiXmlDocument xml;
    xml.Parse(str.c_str());
    for (TiXmlNode* e = xml.FirstChildElement(); e != nullptr; e = e->NextSibling())
    {
        if (e->ValueStr() == "robot")
            return MODEL_URDF;
        if (e->ValueStr() == "sdf")
            return MODEL_SDF;
    }
    fprintf(stderr, "cannot guess robot model format for %s.\n", str.c_str());
    return MODEL_UNKNOWN;
}

pair<string, MODEL_FORMAT> kdl_parser::getRobotModelString(
        string const& model, MODEL_FORMAT format)
{
    char const* sdf_header  = "<sdf";
    char const* urdf_header = "<robot";
    if(model.find(sdf_header) != string::npos)
    {
        if (format == MODEL_URDF)
            throw std::invalid_argument("string contains the '<sdf' marker but the type was MODEL_SDF");
        return make_pair(model, MODEL_SDF);
    }
    else if(model.find(urdf_header) != string::npos)
    {
        if (format == MODEL_SDF)
            throw std::invalid_argument("string contains the '<urdf' marker but the type was MODEL_URDF");
        return make_pair(model, MODEL_URDF);
    }

    if (format == MODEL_AUTO)
        format = guessFormatFromFilename(model);

    ifstream file(model.c_str());
    if(!file)
        throw std::invalid_argument("Robot model file " + model + " does not exist");
    return std::make_pair(
            string((istreambuf_iterator<char>(file)), istreambuf_iterator<char>()),
            format);
}

