EXTRA_DIST = testlibraw.1.in sendiso.1.in dumpiso.1.in isodump.5.in \
	libraw1394.sgml

# man files for testlibraw
man_MANS = testlibraw.1 sendiso.1 dumpiso.1 isodump.5

# libraw1394 docbook documentation

CLEANFILES = libraw1394.aux libraw1394.dvi libraw1394.log libraw1394.out \
	libraw1394.tex libraw1394.pdf

MAINTAINERCLEANFILES = Makefile.in libraw1394.ps

maintainer-clean-local:
	rm -rf libraw1394

doc: psdoc pdfdoc htmldoc
psdoc: libraw1394.ps
pdfdoc: libraw1394.pdf
htmldoc: libraw1394

.PHONY: doc psdoc pdfdoc htmldoc

libraw1394.ps: libraw1394.sgml
	db2ps $<

libraw1394.pdf: libraw1394.sgml
	db2pdf $<

libraw1394: libraw1394.sgml
	db2html $<
#	rm -rf $@
#	mv libraw1394 $@
