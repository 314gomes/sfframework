#ifndef SFFRAMEWORK_EXCEPTIONS_HPP_
#define SFFRAMEWORK_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace sfframework
{

class SkipFrameException : public std::runtime_error
{
public:
  explicit SkipFrameException(const std::string & message)
  : std::runtime_error(message) {}
};

}  // namespace sfframework

#endif  // SFFRAMEWORK_EXCEPTIONS_HPP_