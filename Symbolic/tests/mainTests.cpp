#include <iostream>
#include <cassert>
#include "Symbolic/src/Term.hpp"
#include "Symbolic/src/Symbol.hpp"
#include "Symbolic/src/Bounder.hpp"
#include "Symbolic/src/Constant.hpp"
#include "Symbolic/src/terms.h"

using namespace std;
using namespace Leph::Symbolic;

int main()
{
    Symbol<int>::SymbolPtr t = Symbol<int>::create("t");
    Symbol<int>::SymbolPtr sym1 = Symbol<int>::create("sym1");

    assert(t->toString() == "t");
    assert(sym1->toString() == "sym1");
    assert(sym1->derivate(t)->toString() == "ZERO");
    assert(sym1->derivate(sym1)->toString() == "ONE");
    assert(sym1->substitute<int>(t, t)->toString() == "sym1");

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
    Term<int>::TermPtr term1 = Add<int>::create(sym2, sym3);

    assert(term1->toString() == "(sym2)+(sym3)");
    assert(term1->derivate(sym1)->toString() == "ZERO");
    assert(term1->derivate(t)->toString() == "(d(sym2)/dt)+(d(sym3)/dt)");
    assert(term1->substitute<int>(sym2, t)->toString() == "(t)+(sym3)");
    bounder.setValue(sym2, 1);
    bounder.setValue(sym3, 2);
    assert(term1->evaluate(bounder) == 3);
    term1->reset();
    bounder.setValue(sym2, 2);
    bounder.setValue(sym3, 2);
    assert(term1->evaluate(bounder) == 4);
    
    Term<int>::TermPtr term2 = Mult<int,int,int>::create(sym2, sym3);
    assert(term2->toString() == "(sym2)*(sym3)");
    assert(term2->derivate(sym1)->toString() == "ZERO");
    assert(term2->derivate(t)->toString() 
        == "((d(sym2)/dt)*(sym3))+((sym2)*(d(sym3)/dt))");
    assert(term2->substitute<int>(sym2, term1)->toString() 
        == "((sym2)+(sym3))*(sym3)");

    Symbol<double>::SymbolPtr sym4 = Symbol<double>::create("sym4");
    sym4->depend(t);
    Term<double>::TermPtr term3 = Exp<double,double>::create(sym4);
    assert(term3->derivate(t)->toString() == "(d(sym4)/dt)*(exp(sym4))");

    bounder.setValue(sym4, 1.0);
    assert(term3->evaluate(bounder) > 2.71 
        && term3->evaluate(bounder) < 2.72);

    Term<double>::TermPtr term4 = Pow<double>::create(sym4, 3);
    assert(term4->derivate(t)->toString() 
        == "(d(sym4)/dt)*((3)*((sym4)^2))");
    assert(term4->derivate(t)->derivate(t)->toString() == 
        std::string("((d(d(sym4)/dt)/dt)*((3)*((sym4)^2)))+") 
        + std::string("((d(sym4)/dt)*((3)*((d(sym4)/dt)*((2)*(sym4)))))"));
    assert(term4->substitute<double>(sym4, term3)->toString() 
        == "(exp(sym4))^3");
    assert(term4->substitute<double>(sym3, term3)->toString() 
        == "(sym4)^3");

    Term<double>::TermPtr cst1 = Constant<double>::create(3.14);
    assert(cst1->toString() == "3.14");
    assert(cst1->derivate(t)->toString() == "ZERO");
    assert(cst1->evaluate(bounder) == 3.14);
    assert(cst1->substitute<double>(t, sym4)->toString() == "3.14");

    Term<double>::TermPtr term5 = Frac<double>::create(cst1, sym4);
    assert(term5->toString() == "(3.14)/(sym4)");
    assert(term5->derivate(t)->toString() 
        == "(-((3.14)*(d(sym4)/dt)))/((sym4)^2)");
    assert(term5->substitute<double>(sym4, term3)->toString()
        == "(3.14)/(exp(sym4))");

    Symbol<Leph::Vector::Vector2D<double> >::SymbolPtr sym5 = 
        Symbol<Leph::Vector::Vector2D<double> >::create("vect1");
    sym5->depend(t);
    bounder.setValue(sym5, Leph::Vector::Vector2D<double>(1.0, 2.0));

    Term<double>::TermPtr term6 = 
        X<double,Leph::Vector::Vector2D<double> >::create(sym5);
    assert(term6->toString() == "X(vect1)");
    assert(term6->evaluate(bounder) == 1.0);
    assert(term6->derivate(t)->toString() == "X(d(vect1)/dt)");

    Term<double>::TermPtr term7 = 
        Y<double,Leph::Vector::Vector2D<double> >::create(sym5);
    assert(term7->toString() == "Y(vect1)");
    assert(term7->evaluate(bounder) == 2.0);
    assert(term7->derivate(t)->toString() == "Y(d(vect1)/dt)");

    Term<double>::TermPtr term8 = Minus<double>::create(sym4);
    Term<double>::TermPtr term9 = Minus<double>::create(term8);
    assert(term8->toString() == "-(sym4)");
    assert(term8->derivate(t)->toString() == "-(d(sym4)/dt)");
    assert(term9->toString() == "sym4");

    Term<Leph::Vector::Vector2D<double> >::TermPtr term10 =
        Polar<Leph::Vector::Vector2D<double>,double>::create(sym4);
    assert(term10->toString() == "Polar(sym4)");
    assert(term10->derivate(t)->toString() 
        == "(d(sym4)/dt)*(PolarInv(-(sym4)))");
    
    Term<Leph::Vector::Vector2D<double> >::TermPtr term11 =
        PolarInv<Leph::Vector::Vector2D<double>,double>::create(sym4);
    assert(term11->toString() == "PolarInv(sym4)");
    assert(term11->derivate(t)->toString() 
        == "(d(sym4)/dt)*(Polar(-(sym4)))");
    
    Term<double>::TermPtr term12 = 
        Dot<double,Leph::Vector::Vector2D<double>,
            Leph::Vector::Vector2D<double> >::create(sym5, sym5);
    assert(term12->toString() == "(vect1).(vect1)");
    assert(term12->derivate(sym1)->toString() == "ZERO");
    assert(term12->derivate(t)->toString() 
        == "((d(vect1)/dt).(vect1))+((vect1).(d(vect1)/dt))");
    assert(term12->evaluate(bounder) == 5.0);
    
    Term<Leph::Vector::Vector2D<double> >::TermPtr term13 = 
        Rotation<Leph::Vector::Vector2D<double>, double>::
        create(sym5, sym4);
    assert(term13->toString() 
        == "((X(vect1))*(Polar(sym4)))+((Y(vect1))*(PolarInv(-(sym4))))");

    sym4->reset();
    bounder.setValue(sym4, 0.0);
    assert(term10->evaluate(bounder).x() == 1.0);
    assert(term10->evaluate(bounder).y() == 0.0);

    return 0;
}

