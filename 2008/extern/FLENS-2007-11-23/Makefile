include Makefile.common

all:
	@ echo
	@ echo "   FLENS_HOME=$(FLENS_HOME)"
	@ echo
	@ for i in $(SUBDIRS); \
		do echo; echo "processing dir $$i:"; echo; \
		$(MAKE) -C $$i all; \
	done;
	
clean:
	@ for i in $(SUBDIRS) ; \
		do echo; echo "processing dir $$i:"; echo;\
		$(MAKE) -C $$i clean; \
	done;
	$(RM) -r FLENS-X.X

install:
	$(MKDIR) $(PREFIX)/lib
	$(MKDIR) $(PREFIX)/include/flens/
	cp libflens.$(DYLIB_EXT) $(PREFIX)/lib
	cp flens/*.h flens/*.tcc $(PREFIX)/include/flens/

uninstall:
	$(RM) $(PREFIX)/lib/libflens.$(DYLIB_EXT)
	$(RM) -r $(PREFIX)/include/flens/

dist:
	@$(RM) -r FLENS-X.X
	@./cleanup.sh
	@$(MKDIR) FLENS-X.X
	@cp config.* FLENS-X.X/
	@cp LIC* FLENS-X.X/
	@cp READ* FLENS-X.X/
	@cp INST* FLENS-X.X/
	@cp Makefile FLENS-X.X/
	@cp Makefile.* FLENS-X.X/
	for i in $(SUBDIRS); do  \
		$(MKDIR) FLENS-X.X/$$i;  \
		cp $$i/Makefile FLENS-X.X/$$i/;     \
		cp $$i/*.h FLENS-X.X/$$i/;     \
		cp $$i/*.tcc FLENS-X.X/$$i/;  \
		cp $$i/*.cc FLENS-X.X/$$i/;  \
	done;


