#include <iostream>
#include <cassert>
#include <string>
#include "VectorMap/src/VectorMap.hpp"

using namespace std;
using namespace Leph::VectorMap;

int main()
{
    VectorMap<std::string, int> container;

    assert(container.size() == 0);
    assert(container.isEmpty());
    assert(!container.isKey("a"));

    container.push("a", 42);
    assert(container.size() == 1);
    assert(!container.isEmpty());
    assert(container.isKey("a"));
    assert(container.last() == 42);
    assert(container.lastKey() == "a");
    assert(container.get(0) == 42);
    assert(container[0] == 42);
    assert(container.getKey(0) == "a");
    assert(container.getIndex("a") == 0);
    assert(container.getByKey("a") == 42);

    container[0] = 43;
    assert(container.size() == 1);
    assert(!container.isEmpty());
    assert(container.isKey("a"));
    assert(container.last() == 43);
    assert(container.lastKey() == "a");
    assert(container.get(0) == 43);
    assert(container[0] == 43);
    assert(container.getKey(0) == "a");
    assert(container.getIndex("a") == 0);
    assert(container.getByKey("a") == 43);
    
    container.clear();
    assert(container.size() == 0);
    assert(container.isEmpty());
    assert(!container.isKey("a"));
    
    container.push("a", 42);
    container.push("b", 43);
    assert(container.size() == 2);
    assert(container.isKey("a"));
    assert(container.isKey("b"));
    assert(container["a"] == 42);
    assert(container["b"] == 43);
    assert(container[0] == 42);
    assert(container[1] == 43);

    container.pop();
    assert(container.size() == 1);
    assert(container.isKey("a"));
    assert(!container.isKey("b"));
    assert(container["a"] == 42);
    assert(container[0] == 42);

    container[0] = 43;
    container["a"] = 43;
    container.getByKey("a") = 43;
    assert(container[0] == 43);

    return 0;
}

