#include "XmlConfiguration.h"



XmlConfiguration::XmlConfiguration()
{
	domdoc = 0;
}

XmlConfiguration::XmlConfiguration(const QString& name)
{
	domdoc = new QDomDocument(name);
	loadFile(name);
}

XmlConfiguration::~XmlConfiguration()
{
	delete domdoc;
}

int XmlConfiguration::loadFile(const QString& name)
{
	// recreate document
	delete domdoc;
	domdoc = new QDomDocument(name);

	// load the file
	QFile file(name);
	if (!file.open(QIODevice::ReadOnly))
	{
		printf("Error Loading XML \n");
		return 0;
	}

	// parse the file
	if (!domdoc->setContent(&file))
	{
		file.close();
		printf("Error Parsing XML \n");
		return 0;
	}
	file.close();

	// success
	return 1;
}


int XmlConfiguration::getInt(const QString& element)
{
	// as of now, this only searches one level deep in the dom tree...

	// get root data
	QDomElement root = domdoc->documentElement();
	QDomNode n = root.firstChild();

	// loop through all elements and check for a match
	while (!n.isNull())
	{
		QDomElement e = n.toElement();
		if (!e.isNull())
		{
			if (element == e.tagName() )
			{
				QString s = e.text();
				return s.toInt();
			}
		}
		// move to next
		n = n.nextSibling();
	}

	// not matched
	return -1;
}

float XmlConfiguration::getFloat(const QString& element)
{
	// as of now, this only searches one level deep in the dom tree...

	// get root data
	QDomElement root = domdoc->documentElement();
	QDomNode n = root.firstChild();

	// loop through all elements and check for a match
	while (!n.isNull())
	{
		QDomElement e = n.toElement();
		if (!e.isNull())
		{
			if (element == e.tagName() )
			{
				QString s = e.text();
				return s.toFloat();
			}
		}
		// move to next
		n = n.nextSibling();
	}

	// not matched
	return -1;
}


