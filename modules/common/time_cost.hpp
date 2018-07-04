/**
 * <p>Macros:</p>
 * <ul>
 * <li>TIME_[BEG|END]: Time beg, end
 * <li>TIME_[BEG|END]_FUNC[2]: Func time beg, end
 * <li>TIME_COST: Turn TIME_* on
 * </ul>
 */
#ifndef TIME_COST_HPP_
#define TIME_COST_HPP_
#pragma once

#include <cmath>
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>

#include "log.hpp"
#include "time.hpp"

#ifdef OS_ANDROID
#include <tr1/unordered_map>
#else
#include <unordered_map>
#endif

namespace xt {

class TimeCost {
public:
#ifdef OS_ANDROID
    using tag_map_t = std::tr1::unordered_map<std::string, std::shared_ptr<TimeCost>>;
#else
    using tag_map_t = std::unordered_map<std::string, std::shared_ptr<TimeCost>>;
#endif

    explicit TimeCost(const std::string &tag) : tag_(tag) {}
    ~TimeCost() {}

    std::ostream &ToString(std::ostream &os) const {
        float ms = xt::count<std::chrono::microseconds>(elapsed()) * 0.001f;
        os << tag_ << std::endl
            << "BEG: " << xt::to_local_string(beg_) << std::endl
            << "END: " << xt::to_local_string(end_) << std::endl
            << "COST: " << ms << "ms";
        return os;
    }

    std::ostream &ToLineString(std::ostream &os) const {
        float ms = xt::count<std::chrono::microseconds>(elapsed()) * 0.001f;
        os << tag_ << ": " << ms << "ms, "
            << xt::to_local_string(beg_, "%T") << " > "
            << xt::to_local_string(end_, "%T");
        return os;
    }

    std::string ToString() const {
        std::stringstream ss;
        ToString(ss);
        return ss.str();
    }

    std::string ToLineString() const {
        std::stringstream ss;
        ToLineString(ss);
        return ss.str();
    }

    std::string tag() const { return tag_; }
    xt::system_clock::time_point beg() const { return beg_; }
    xt::system_clock::time_point end() const { return end_; }
    xt::system_clock::duration elapsed() const { return end_ - beg_; }

    void set_beg(const xt::system_clock::time_point &t) { beg_ = t; }
    void set_end(const xt::system_clock::time_point &t) { end_ = t; }

    static std::shared_ptr<TimeCost> Beg(const std::string &tag) {
        const std::lock_guard<std::mutex> lock(GetMutex());
        std::shared_ptr<TimeCost> cost = std::make_shared<TimeCost>(tag);
        cost->beg_ = xt::now();
        tag_map_t &map = GetTagMap();
        auto it = map.insert({tag, cost});
        if (!it.second) {
            LOGE("This tag already in use");
            throw std::logic_error("This tag already in use");
        }
        return it.first->second;
    }

    static std::shared_ptr<TimeCost> End(const std::string &tag) {
        const std::lock_guard<std::mutex> lock(GetMutex());
        tag_map_t &map = GetTagMap();
        auto it = map.find(tag);
        if (it == map.end()) {
            LOGE("This tag not Beg before End");
            throw std::logic_error("This tag not Beg before End");
        }
        std::shared_ptr<TimeCost> cost = it->second;
        cost->end_ = xt::now();
        map.erase(it);
        return cost;
    }

private:
    std::string tag_;
    xt::system_clock::time_point beg_;
    xt::system_clock::time_point end_;

    static tag_map_t  &GetTagMap() { static tag_map_t map;  return map; }
    static std::mutex &GetMutex()  { static std::mutex mtx; return mtx; }
};

}  // namespace xt

#ifdef TIME_COST
  #define TIME_BEG(tag) xt::TimeCost::Beg(tag)
  #define TIME_END(tag) LOGI(xt::TimeCost::End(tag)->ToLineString())
  #define TIME_BEG_FUNC(tag) do { \
    std::stringstream ss; \
    ss << __func__ << "::" << tag; \
    TIME_BEG(ss.str()); \
  } while (0)
  #define TIME_END_FUNC(tag) do { \
    std::stringstream ss; \
    ss << __func__ << "::" << tag; \
    TIME_END(ss.str()); \
  } while (0)
  #define TIME_BEG_FUNC2 TIME_BEG(__func__)
  #define TIME_END_FUNC2 TIME_END(__func__)
#else
  #define TIME_BEG(tag)
  #define TIME_END(tag)
  #define TIME_BEG_FUNC(tag)
  #define TIME_END_FUNC(tag)
  #define TIME_BEG_FUNC2
  #define TIME_END_FUNC2
#endif

#endif  // TIME_COST_HPP_
