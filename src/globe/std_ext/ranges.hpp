#ifndef GLOBEART_SRC_GLOBE_HELPERS_RANGES_HPP_
#define GLOBEART_SRC_GLOBE_HELPERS_RANGES_HPP_

#include <ranges>
#include <iterator>

namespace globe {

template<std::ranges::range RangeType>
auto circular_adjacent_pairs(const RangeType &range) {
    struct CircularPairs {
        const RangeType &range_ref;

        struct Iterator {
            using iterator_t = std::ranges::iterator_t<const RangeType>;
            iterator_t it;
            iterator_t begin_it;
            iterator_t end_it;
            bool is_wraparound;

            using value_type = std::pair<
                std::ranges::range_reference_t<const RangeType>,
                std::ranges::range_reference_t<const RangeType>
            >;

            value_type operator*() const {
                if (is_wraparound) {
                    iterator_t last = begin_it;
                    for (auto temp = begin_it; temp != end_it; ++temp) {
                        last = temp;
                    }
                    return {*last, *begin_it};
                } else {
                    auto next = std::next(it);
                    return {*it, *next};
                }
            }

            Iterator &operator++() {
                if (is_wraparound) {
                    it = end_it;
                    is_wraparound = false;
                } else {
                    ++it;
                    auto next = std::next(it);
                    if (next == end_it) {
                        is_wraparound = true;
                    }
                }
                return *this;
            }

            bool operator==(const Iterator &other) const {
                return it == other.it && is_wraparound == other.is_wraparound;
            }

            bool operator!=(const Iterator &other) const {
                return !(*this == other);
            }
        };

        Iterator begin() const {
            auto it = std::ranges::begin(range_ref);
            auto end_it = std::ranges::end(range_ref);
            bool is_wraparound = false;

            if (it == end_it) {
                return {end_it, it, end_it, false};
            }

            auto next = std::next(it);
            if (next == end_it) {
                is_wraparound = true;
            }

            return {it, it, end_it, is_wraparound};
        }

        Iterator end() const {
            auto end_it = std::ranges::end(range_ref);
            return {end_it, std::ranges::begin(range_ref), end_it, false};
        }
    };

    return CircularPairs{range};
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_HELPERS_RANGES_HPP_

