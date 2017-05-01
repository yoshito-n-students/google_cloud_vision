#ifndef GOOGLE_CLOUD_VISION_JSON_PARSER
#define GOOGLE_CLOUD_VISION_JSON_PARSER

#include <iostream>
#include <sstream>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <google_cloud_vision/namespace.hpp>
#include <google_cloud_vision_srvs/Annotate.h>

namespace google_cloud_vision {

//
// conversions from boost::property_tree::ptree
//

// for types boost.property_tree natively supports
template < typename Value > void fromPtree(const bp::ptree &ptree, Value &value) {
  value = ptree.get_value< Value >();
}

template < typename Value > void fromPtree(const bp::ptree &, std::vector< Value > &);

static inline void fromPtree(const bp::ptree &ptree, msgs::Vertex &vertex) {
  vertex = msgs::Vertex();
  // all fields are required
  fromPtree(ptree.get_child("x"), vertex.x);
  fromPtree(ptree.get_child("y"), vertex.y);
}

static inline void fromPtree(const bp::ptree &ptree, msgs::BoundingPoly &poly) {
  poly = msgs::BoundingPoly();
  // "vertices" is a required field
  fromPtree(ptree.get_child("vertices"), poly.vertices);
}

static inline void fromPtree(const bp::ptree &ptree, msgs::LatLng &lat_lng) {
  lat_lng = msgs::LatLng();
  // all fields are required
  fromPtree(ptree.get_child("latitude"), lat_lng.latitude);
  fromPtree(ptree.get_child("longitude"), lat_lng.longitude);
}

static inline void fromPtree(const bp::ptree &ptree, msgs::LocationInfo &info) {
  info = msgs::LocationInfo();
  // "latLng" is a required field
  fromPtree(ptree.get_child("latLng"), info.lat_lng);
}

static inline void fromPtree(const bp::ptree &ptree, msgs::Property &property) {
  property = msgs::Property();
  if (ptree.count("name") > 0) {
    fromPtree(ptree.get_child("name"), property.name);
  }
  if (ptree.count("value") > 0) {
    fromPtree(ptree.get_child("value"), property.value);
  }
  if (ptree.count("uint64Value") > 0) {
    property.has_uint64_value = true;
    fromPtree(ptree.get_child("uint64Value"), property.uint64_value);
  }
}

static inline void fromPtree(const bp::ptree &ptree, msgs::EntityAnnotation &annotation) {
  annotation = msgs::EntityAnnotation();
  if (ptree.count("mid") > 0) {
    fromPtree(ptree.get_child("mid"), annotation.mid);
  }
  if (ptree.count("locale") > 0) {
    fromPtree(ptree.get_child("locale"), annotation.locale);
  }
  if (ptree.count("description") > 0) {
    fromPtree(ptree.get_child("description"), annotation.description);
  }
  if (ptree.count("score") > 0) {
    annotation.has_score = true;
    fromPtree(ptree.get_child("score"), annotation.score);
  }
  if (ptree.count("confidence") > 0) {
    annotation.has_confidence = true;
    fromPtree(ptree.get_child("confidence"), annotation.confidence);
  }
  if (ptree.count("topicality") > 0) {
    annotation.has_topicality = true;
    fromPtree(ptree.get_child("topicality"), annotation.topicality);
  }
  if (ptree.count("boundingPoly") > 0) {
    annotation.has_bounding_poly = true;
    fromPtree(ptree.get_child("boundingPoly"), annotation.bounding_poly);
  }
  if (ptree.count("locations") > 0) {
    fromPtree(ptree.get_child("locations"), annotation.locations);
  }
  if (ptree.count("properties") > 0) {
    fromPtree(ptree.get_child("properties"), annotation.properties);
  }
}

static inline void fromPtree(const bp::ptree &ptree, msgs::Status &status) {
  status = msgs::Status();
  if (ptree.count("code") > 0) {
    status.has_code = true;
    fromPtree(ptree.get_child("code"), status.code);
  }
  if (ptree.count("message") > 0) {
    fromPtree(ptree.get_child("message"), status.message);
  }
  if (ptree.count("details") > 0) {
    // subfields in "details" are arbitrary. so convert them into text format.
    std::ostringstream oss;
    bp::json_parser::write_json(oss, ptree.get_child("details"));
    status.details = oss.str();
  }
}

static inline void fromPtree(const bp::ptree &ptree, msgs::AnnotateImageResponse &response) {
  response = msgs::AnnotateImageResponse();
  if (ptree.count("landmarkAnnotations") > 0) {
    fromPtree(ptree.get_child("landmarkAnnotations"), response.landmark_annotations);
  }
  if (ptree.count("logoAnnotations") > 0) {
    fromPtree(ptree.get_child("logoAnnotations"), response.logo_annotations);
  }
  if (ptree.count("labelAnnotations") > 0) {
    fromPtree(ptree.get_child("labelAnnotations"), response.label_annotations);
  }
  if (ptree.count("textAnnotations") > 0) {
    fromPtree(ptree.get_child("textAnnotations"), response.text_annotations);
  }
  if (ptree.count("error") > 0) {
    response.has_error = true;
    fromPtree(ptree.get_child("error"), response.error);
  }
}

static inline void fromPtree(const bp::ptree &ptree, srvs::Annotate::Response &response) {
  response = srvs::Annotate::Response();
  // "responses" is a required field
  fromPtree(ptree.get_child("responses"), response.responses);
}

template < typename Value > void fromPtree(const bp::ptree &ptree, std::vector< Value > &values) {
  values.clear();
  for (bp::ptree::const_iterator child = ptree.begin(); child != ptree.end(); ++child) {
    if (!child->first.empty()) {
      continue;
    }
    values.push_back(Value());
    fromPtree(child->second, values.back());
  }
}

//
// JSON reader
//

template < typename Value > void readJson(std::istream &is, Value &value) {
  bp::ptree ptree;
  bp::json_parser::read_json(is, ptree);
  fromPtree(ptree, value);
}

//
// conversions to boost::property_tree::ptree
//

// for types boost.property_tree natively supports
template < typename Value > bp::ptree toPtree(const Value &value) {
  bp::ptree ptree;
  ptree.put_value(value);
  return ptree;
}

template < typename Value > bp::ptree toPtree(const std::vector< Value > &);

static inline bp::ptree toPtree(const msgs::ImageSource &source) {
  bp::ptree ptree;
  if (!source.gcs_image_uri.empty()) {
    ptree.put_child("gcsImageUri", toPtree(source.gcs_image_uri));
  }
  if (!source.image_uri.empty()) {
    ptree.put_child("imageUri", toPtree(source.image_uri));
  }
  return ptree;
}

static inline bp::ptree toPtree(const msgs::Image &image) {
  bp::ptree ptree;
  if (!image.content.empty()) {
    ptree.put_child("content", toPtree(image.content));
  }
  if (image.has_source) {
    ptree.put_child("source", toPtree(image.source));
  }
  return ptree;
}

static inline bp::ptree toPtree(const msgs::Feature &feature) {
  bp::ptree ptree;
  // "type" is a required field
  ptree.put_child("type", toPtree(feature.type));
  if (feature.has_max_results) {
    ptree.put_child("maxResults", toPtree(feature.max_results));
  }
  return ptree;
}

static inline bp::ptree toPtree(const msgs::LatLng &lat_lng) {
  bp::ptree ptree;
  // all fields are required
  ptree.put_child("latitude", toPtree(lat_lng.latitude));
  ptree.put_child("longitude", toPtree(lat_lng.longitude));
  return ptree;
}

static inline bp::ptree toPtree(const msgs::LatLongRect &rect) {
  bp::ptree ptree;
  if (rect.has_min_lat_lng) {
    ptree.put_child("minLatLng", toPtree(rect.min_lat_lng));
  }
  if (rect.has_max_lat_lng) {
    ptree.put_child("maxLatLng", toPtree(rect.max_lat_lng));
  }
  return ptree;
}

static inline bp::ptree toPtree(const msgs::CropHintsParams &params) {
  bp::ptree ptree;
  // "aspectRatios" is a required field
  ptree.put_child("aspectRatios", toPtree(params.aspect_ratios));
  return ptree;
}

static inline bp::ptree toPtree(const msgs::ImageContext &context) {
  bp::ptree ptree;
  if (context.has_lat_long_rect) {
    ptree.put_child("latLongRect", toPtree(context.lat_long_rect));
  }
  if (!context.language_hints.empty()) {
    ptree.put_child("languageHints", toPtree(context.language_hints));
  }
  if (context.has_crop_hints_params) {
    ptree.put_child("cropHintsParams", toPtree(context.crop_hints_params));
  }
  return ptree;
}

static inline bp::ptree toPtree(const msgs::AnnotateImageRequest &request) {
  bp::ptree ptree;
  // "image" is a required field
  ptree.put_child("image", toPtree(request.image));
  // "features" is a required field
  ptree.put_child("features", toPtree(request.features));
  if (request.has_image_context) {
    ptree.put_child("imageContext", toPtree(request.image_context));
  }
  return ptree;
}

static inline bp::ptree toPtree(const srvs::Annotate::Request &request) {
  bp::ptree ptree;
  // "requests" is a required field
  ptree.put_child("requests", toPtree(request.requests));
  return ptree;
}

template < typename Value > bp::ptree toPtree(const std::vector< Value > &values) {
  bp::ptree ptree;
  for (typename std::vector< Value >::const_iterator value = values.begin(); value != values.end();
       ++value) {
    ptree.push_back(bp::ptree::value_type("", toPtree(*value)));
  }
  return ptree;
}

//
// JSON writer
//

template < typename Value >
void writeJson(std::ostream &os, const Value &value, const bool pretty = true) {
  bp::json_parser::write_json(os, toPtree(value), pretty);
}
}

#endif