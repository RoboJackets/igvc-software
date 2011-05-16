#ifndef XMLCONFIGURATION_H_
#define XMLCONFIGURATION_H_

#include <qfile.h>
#include <qxml.h>
#include <QDomElement>

/*
 * This file provides an interface for the XML classes in Qt4
 * 	 by: Chris McClanahan
 *
 *
 * Usage:
 *
 *		XmlConfiguration cfg("Config.xml");
 * 		int variable = cfg.getInt("variable");
 *
 * 		where 'Variable' is in a file Config.xml file such as:
 *
 * 				<?xml version="1.0" encoding="UTF-8"?>
 * 				<Config>
 * 					<variable>123</variable>
 * 				</Config>
 *
 * 		Note: getInt() only goes one level deep in the tree!
 */

class XmlConfiguration
{
public:
	XmlConfiguration();
	XmlConfiguration(const QString& name);
	virtual ~XmlConfiguration();

	// xml document dom tree object
	QDomDocument* domdoc;
	// load xml file (needs .xml extension)
	int loadFile(const QString& name);
	// returns an integer at specified location in tree
	int getInt(const QString& element);
	// returns a floating point number at specified location in tree
	float getFloat(const QString& element);

};

#endif /*XMLCONFIGURATION_H_*/
