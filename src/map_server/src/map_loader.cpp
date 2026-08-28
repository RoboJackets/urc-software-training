// map_loader.cpp
//
// Parses the map metadata yaml, reads the PGM image (binary P5 or ASCII P2),
// and builds a nav_msgs/OccupancyGrid using standard ROS map_server semantics.

#include "map_server/map_loader.hpp"

#include <cctype>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>

namespace robonav_training
{

namespace
{

struct PgmImage
{
  unsigned int width = 0;
  unsigned int height = 0;
  int maxval = 255;
  std::vector<int> pixels;  // row-major, first row is the TOP of the image
};

// Returns the directory part of a path (with trailing '/'), or "" if none.
std::string dirname(const std::string & path)
{
  const std::size_t slash = path.find_last_of('/');
  if (slash == std::string::npos) {
    return "";
  }
  return path.substr(0, slash + 1);
}

// Reads the next PGM header token, skipping whitespace and '#' comment lines.
bool readHeaderToken(std::ifstream & in, std::string & token)
{
  int c;
  while ((c = in.get()) != EOF) {
    if (c == '#') {
      while ((c = in.get()) != EOF && c != '\n') {
      }
    } else if (!std::isspace(c)) {
      break;
    }
  }
  if (c == EOF) {
    return false;
  }
  token.clear();
  do {
    token.push_back(static_cast<char>(c));
    c = in.get();
  } while (c != EOF && !std::isspace(c));
  return true;
}

// Parses a PGM file (binary P5 or ASCII P2). Throws std::runtime_error on any error.
void readPgm(const std::string & path, PgmImage & img)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("Could not open image file: " + path);
  }

  std::string magic;
  if (!readHeaderToken(in, magic) || (magic != "P5" && magic != "P2")) {
    throw std::runtime_error("Image is not a P5/P2 PGM: " + path);
  }

  std::string w_str, h_str, max_str;
  if (!readHeaderToken(in, w_str) || !readHeaderToken(in, h_str) ||
    !readHeaderToken(in, max_str))
  {
    throw std::runtime_error("Malformed PGM header: " + path);
  }

  img.width = static_cast<unsigned int>(std::stoul(w_str));
  img.height = static_cast<unsigned int>(std::stoul(h_str));
  img.maxval = std::stoi(max_str);
  if (img.width == 0 || img.height == 0 || img.maxval <= 0) {
    throw std::runtime_error("PGM has invalid dimensions/maxval: " + path);
  }

  const std::size_t count = static_cast<std::size_t>(img.width) * img.height;
  img.pixels.resize(count);

  if (magic == "P5") {
    // P5: data is `count` raw bytes; readHeaderToken consumed the single separator (maxval <= 255).
    std::vector<unsigned char> raw(count);
    in.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(count));
    if (static_cast<std::size_t>(in.gcount()) != count) {
      throw std::runtime_error("PGM data shorter than expected: " + path);
    }
    for (std::size_t i = 0; i < count; ++i) {
      img.pixels[i] = raw[i];
    }
  } else {
    // P2: data is `count` whitespace-separated ASCII integers.
    for (std::size_t i = 0; i < count; ++i) {
      if (!(in >> img.pixels[i])) {
        throw std::runtime_error("PGM data shorter than expected: " + path);
      }
    }
  }
}

}  // namespace

nav_msgs::msg::OccupancyGrid loadMapFromYaml(const std::string & yaml_path)
{
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    throw std::runtime_error(
            "Failed to parse map yaml '" + yaml_path + "': " + e.what());
  }

  std::string image;
  double resolution = 0.0;
  std::vector<double> origin;
  int negate = 0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.25;
  try {
    image = doc["image"].as<std::string>();
    resolution = doc["resolution"].as<double>();
    origin = doc["origin"].as<std::vector<double>>();
    negate = doc["negate"].as<int>();
    occupied_thresh = doc["occupied_thresh"].as<double>();
    free_thresh = doc["free_thresh"].as<double>();
  } catch (const std::exception & e) {
    throw std::runtime_error(
            std::string("Missing/invalid field in map yaml: ") + e.what());
  }
  if (origin.size() != 3) {
    throw std::runtime_error("Map 'origin' must have 3 elements [x, y, yaw].");
  }

  // The image path in the yaml is relative to the yaml file's directory.
  const std::string image_path = dirname(yaml_path) + image;

  PgmImage img;
  readPgm(image_path, img);

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.width = img.width;
  grid.info.height = img.height;
  grid.info.origin.position.x = origin[0];
  grid.info.origin.position.y = origin[1];
  grid.info.origin.position.z = 0.0;

  // origin[2] is a yaw (rotation about Z); convert it to a quaternion.
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, origin[2]);
  grid.info.origin.orientation = tf2::toMsg(q);

  grid.data.resize(static_cast<std::size_t>(img.width) * img.height);

  const double maxval = static_cast<double>(img.maxval);
  for (unsigned int row = 0; row < img.height; ++row) {
    // Classic bug to avoid: OccupancyGrid data[0] is bottom-left, but PGM row 0 is the top.
    const unsigned int img_row = img.height - 1 - row;
    for (unsigned int col = 0; col < img.width; ++col) {
      const int pixel = img.pixels[static_cast<std::size_t>(img_row) * img.width + col];

      // ROS map_server semantics: darker pixel => higher occupancy.
      const double occ = (negate ? pixel : (img.maxval - pixel)) / maxval;
      int8_t value;
      if (occ > occupied_thresh) {
        value = 100;        // occupied
      } else if (occ < free_thresh) {
        value = 0;          // free
      } else {
        value = -1;         // unknown
      }
      grid.data[static_cast<std::size_t>(row) * img.width + col] = value;
    }
  }

  return grid;
}

}  // namespace robonav_training
