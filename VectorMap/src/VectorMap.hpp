#ifndef LEPH_VECTORMAP_VECTORMAP_HPP
#define LEPH_VECTORMAP_VECTORMAP_HPP

#include <vector>
#include <map>
#include <stdexcept>

namespace Leph {
namespace VectorMap {

/**
 * VectorMap
 *
 * This utility class mix
 * standard Vector and Map container
 * Direct access to
 * index -> element
 * index -> key
 * key -> element
 * key -> index
 */
template <class Key, class Elt>
class VectorMap
{
    public:

        /**
         * Initialization
         */
        VectorMap() :
            _elementVector(),
            _keyVector(),
            _indexMap()
        {
        }

        /**
         * Clear internal data
         */
        inline void clear()
        {
            _elementVector.clear();
            _keyVector.clear();
            _indexMap.clear();
        }

        /**
         * Return the number of contained associations
         */
        inline size_t size() const
        {
            return _elementVector.size();
        }

        /**
         * Return true if the container is empty
         */
        inline bool isEmpty() const
        {
            return (size() == 0);
        }

        /**
         * Return true if the given key is contained
         */
        inline bool isKey(const Key& key) const
        {
            return (_indexMap.count(key) != 0);
        }

        /**
         * Insert (at back) an new association in the containers
         */
        inline void push(const Key& key, const Elt& elt)
        {
            if (isKey(key)) {
                throw std::logic_error("VectorMap key already inserted");
            }

            _elementVector.push_back(elt);
            _keyVector.push_back(key);
            _indexMap[key] = _elementVector.size()-1;
        }

        /**
         * Return the last inserted element and key
         */
        inline const Elt& last() const
        {
            if (isEmpty()) {
                throw std::logic_error("VectorMap (last) is empty");
            }

            return _elementVector[size()-1];
        }
        inline const Key& lastKey() const
        {
            if (isEmpty()) {
                throw std::logic_error("VectorMap (lastKey) is empty");
            }

            return _keyVector[size()-1];
        }

        /**
         * Remove the last inserted association in the container
         */
        inline void pop()
        {
            if (isEmpty()) {
                throw std::logic_error("VectorMap (pop) is empty");
            }

            const Key& key = lastKey();
            _indexMap.erase(key);
            _keyVector.pop_back();
            _elementVector.pop_back();
        }

        /**
         * Index to element
         */
        inline const Elt& get(size_t index) const
        {
            return _elementVector[index];
        }
        inline Elt& get(size_t index)
        {
            return _elementVector[index];
        }
        inline const Elt& at(size_t index) const
        {
            return _elementVector.at(index);
        }
        inline Elt& at(size_t index)
        {
            return _elementVector(index);
        }
        inline const Elt& operator[](size_t index) const
        {
            return _elementVector[index];
        }
        inline Elt& operator[](size_t index)
        {
            return _elementVector[index];
        }

        /**
         * Index to key
         */
        inline const Key& getKey(size_t index) const
        {
            return _keyVector[index];
        }
        inline const Key& atKey(size_t index) const
        {
            return _keyVector.at(index);
        }

        /**
         * Key to index
         */
        inline size_t getIndex(const Key& key) const
        {
            return _indexMap.at(key);
        }

        /**
         * Key to element
         */
        inline const Elt& getByKey(const Key& key) const
        {
            return _elementVector[_indexMap.at(key)];
        }
        inline Elt& getByKey(const Key& key)
        {
            return _elementVector[_indexMap.at(key)];
        }
        inline const Elt& operator[](const Key& key) const
        {
            return _elementVector[_indexMap.at(key)];
        }
        inline Elt& operator[](const Key& key)
        {
            return _elementVector[_indexMap.at(key)];
        }

    private:

        /**
         * Container for index to elements association
         */
        std::vector<Elt> _elementVector;

        /**
         * Container for index to key association
         */
        std::vector<Key> _keyVector;

        /**
         * Container for key to index association
         */
        std::map<Key, size_t> _indexMap;
};

}
}

#endif

