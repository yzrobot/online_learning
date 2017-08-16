#ifndef ASSO_EXCEPTION_H
#define ASSO_EXCEPTION_H

#include <iostream>
#include <exception>

using namespace std;

struct asso_exception: public exception
{
  const char* what() const throw()
  {
    return "Unknown association algorithm!";
  }
};

struct observ_exception: public exception
{
  const char* what() const throw()
  {
    return "Unknown observation model!";
  }
};

struct filter_exception: public exception
{
  const char* what() const throw()
  {
    return "Unknown filter type!";
  }
};

#endif // ASSO_EXCEPTION_H
