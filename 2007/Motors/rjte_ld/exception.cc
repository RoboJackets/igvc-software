#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "exception.h"

text_exception::text_exception(const char *text)
{
	this->text = text;
}

const char *text_exception::what() const throw()
{
	return text;
}

////////

errno_exception::errno_exception(const char *description)
{
	const char *err = strerror(errno);
	int len = strlen(description) + 3 + strlen(err);

	text = new char[len];
	if (text)
	{
		snprintf(text, len, "%s: %s", description, err);
		text[len - 1] = 0;
	}
}

errno_exception::~errno_exception() throw()
{
	delete text;
}

const char *errno_exception::what() const throw()
{
	return text;
}

