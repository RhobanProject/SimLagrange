#include <iostream>
#include <cassert>
#include "Symbolic/src/Term.hpp"
#include "Symbolic/src/BaseSymbol.hpp"
#include "Symbolic/src/Symbol.hpp"
#include "Symbolic/src/Bounder.hpp"
#include "Symbolic/src/UnaryFunction.hpp"
#include "Symbolic/src/BinaryFunction.hpp"
#include "Symbolic/src/terms/Add.hpp"
#include "Symbolic/src/terms/Mult.hpp"

using namespace std;
using namespace Leph::Symbolic;

int main()
{
    Symbol<int>::SymbolPtr t = Symbol<int>::create("t");
    Symbol<int>::SymbolPtr sym1 = Symbol<int>::create("sym1");

    assert(t->toString() == "t");
    assert(sym1->toString() == "sym1");
    assert(sym1->derivate(t)->toString() == "ZERO");

    sym1->reset();
    sym1->depend(t);
    assert(sym1->derivate(t)->toString() == "d(sym1)/dt");

    Bounder bounder;
    bounder.setValue(sym1, 42);
    assert(sym1->evaluate(bounder) == 42);
    bounder.setValue(sym1, 43);
    sym1->reset();
    assert(sym1->evaluate(bounder) == 43);

    Symbol<int>::SymbolPtr sym2 = Symbol<int>::create("sym2");
    Symbol<int>::SymbolPtr sym3 = Symbol<int>::create("sym3");
    sym2->depend(t);
    sym3->depend(t);
    Term<int>::TermPtr term1 = Add<int,int,int>::create(sym2, sym3);

    assert(term1->toString() == "(sym2)+(sym3)");
    assert(term1->derivate(sym1)->toString() == "ZERO");
    assert(term1->derivate(t)->toString() == "(d(sym2)/dt)+(d(sym3)/dt)");
    bounder.setValue(sym2, 1);
    bounder.setValue(sym3, 2);
    assert(term1->evaluate(bounder) == 3);
    
    Term<int>::TermPtr term2 = Mult<int,int,int>::create(sym2, sym3);
    assert(term2->toString() == "(sym2)*(sym3)");
    assert(term2->derivate(sym1)->toString() == "ZERO");
    assert(term2->derivate(t)->toString() 
        == "((d(sym2)/dt)*(sym3))+((sym2)*(d(sym3)/dt))");

    return 0;
}

