include $(top_srcdir)/common/common.am

bin_PROGRAMS = Project

Project_SOURCES = Project.c
Project_CFLAGS = $(MODBUS_CFLAGS) $(BACNET_CFLAGS)
Project_LDADD = $(MODBUS_LIBS) $(BACNET_LIBS)

BUILT_SOURCES = bacnet_namespace.h .bacnet_dependent_src_stamp
EXTRA_DIST = bacnet_api_names

.bacnet_dependent_src_stamp: bacnet_api_names $(BACNET_DEPENDENT_SRC)
	cat $< | $(top_srcdir)/scripts/check_namespace.sh $< $?
	touch $@

bacnet_namespace.h: bacnet_api_names
	cat $< | $(top_srcdir)/scripts/build_namespace.sh > $@
