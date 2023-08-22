#
# Copyright (c) 2019 LAAS/CNRS
# All rights reserved.
#
# Redistribution  and  use  in  source  and binary  forms,  with  or  without
# modification, are permitted provided that the following conditions are met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
# THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
# WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
# MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
# ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
# WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
# IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#                                             Martin Jacquet - September 2022
#
# Modified by: Selvakumar H S - August 2023
#

lib_LTLIBRARIES = librealsense_codels.la

librealsense_codels_la_SOURCES  =	realsense_c_types.h
librealsense_codels_la_SOURCES +=	realsense_codels.cc
librealsense_codels_la_SOURCES +=	realsense_main_codels.cc
librealsense_codels_la_SOURCES +=	realsense_visual_codels.cc
librealsense_codels_la_SOURCES +=	realsense_inertial_codels.cc
librealsense_codels_la_SOURCES +=	realsense_depth_codels.cc
librealsense_codels_la_SOURCES +=	rs.cc

librealsense_codels_la_CPPFLAGS =	$(requires_CFLAGS)
librealsense_codels_la_LIBADD   =	$(requires_LIBS)
librealsense_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
librealsense_codels_la_LIBADD  +=	$(codels_requires_LIBS)
librealsense_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	realsense_c_types.h
CLEANFILES=	${BUILT_SOURCES}

realsense_c_types.h: ${top_srcdir}/realsense.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/realsense_c_types.h.d
