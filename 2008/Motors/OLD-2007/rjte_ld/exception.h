#ifndef _EXCEPTION_H_
#define _EXCEPTION_H_

#include <exception>

class text_exception: public std::exception
{
public:
	text_exception(const char *text);

	const char *what() const throw();

protected:
	const char *text;
};

class errno_exception: public std::exception
{
public:
	errno_exception(const char *description);
	~errno_exception() throw();

	const char *what() const throw();

protected:
	char *text;
};

#endif // _EXCEPTION_H_

