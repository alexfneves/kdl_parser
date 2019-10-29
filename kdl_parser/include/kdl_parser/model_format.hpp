#ifndef KDL_PARSER__MODEL_FORMAT_HPP
#define KDL_PARSER__MODEL_FORMAT_HPP

#include <string>
#include <utility>
#include <tinyxml.h>  // NOLINT

namespace kdl_parser{

//define format of robot model
enum MODEL_FORMAT
{
    //! unknown file format
    MODEL_UNKNOWN,
    //! try to guess the format
    MODEL_AUTO,
    //! URDF model
    MODEL_URDF,
    //! SDF model
    MODEL_SDF
};

/** Returns the model format based on a filename's extension
 */
MODEL_FORMAT guessFormatFromFilename(const std::string& file);

/** Returns the model format based on a filename's extension
 */
MODEL_FORMAT guessFormatFromString(const std::string& str);

/** Returns the name of one of the ROBOT_MODEL_* format IDs
 */
const char* formatNameFromID(int type);

/** Resolve a string that is either a XML robot model or a path to a file into
 * the full XML string and its format
 *
 * @param model either a path to a model file, or the full XML model.
 *   In the latter case, it is expected to start with the XML preamble
 *   (<?xml ...?>).
 * @param format the model format. ROBOT_MODEL_AUTO can only be used in
 *   the case of a path, the function does not know how to auto-detect
 *   the format of a XML string yet.
 * @return the XML string and its format
 */
std::pair<std::string, MODEL_FORMAT> getRobotModelString(
        const std::string& model, MODEL_FORMAT format);
}

#endif
