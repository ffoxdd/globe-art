#ifndef GLOBEART_SRC_GLOBE_VORONOI_CORE_HANDLE_ITERATOR_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_CORE_HANDLE_ITERATOR_HPP_

#include <iterator>
#include <cstddef>

namespace globe {

template<typename Iterator, typename ValueType>
class HandleIterator {
 public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = ValueType;
    using difference_type = std::ptrdiff_t;
    using pointer = const ValueType *;
    using reference = const ValueType &;

    HandleIterator() : _iter() { }
    explicit HandleIterator(Iterator iter) : _iter(iter) { }

    reference operator*() const { return _iter; }
    pointer operator->() const { return &_iter; }

    HandleIterator &operator++() {
        ++_iter;
        return *this;
    }

    HandleIterator operator++(int) {
        HandleIterator tmp = *this;
        ++(*this);
        return tmp;
    }

    friend bool operator==(const HandleIterator &a, const HandleIterator &b) { return a._iter == b._iter; }
    friend bool operator!=(const HandleIterator &a, const HandleIterator &b) { return a._iter != b._iter; }

 private:
    Iterator _iter;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_VORONOI_CORE_HANDLE_ITERATOR_HPP_

