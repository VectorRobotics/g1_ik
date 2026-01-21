/*
 *    This file is part of CasADi.
 *
 *    CasADi -- A symbolic framework for dynamic optimization.
 *    Copyright (C) 2010-2023 Joel Andersson, Joris Gillis, Moritz Diehl,
 *                            KU Leuven. All rights reserved.
 *    Copyright (C) 2011-2014 Greg Horn
 *
 *    CasADi is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    CasADi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with CasADi; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef CASADI_CONFIG_H // NOLINT(build/header_guard)
#define CASADI_CONFIG_H // NOLINT(build/header_guard)

#define CASADI_MAJOR_VERSION 3
#define CASADI_MINOR_VERSION 6
#define CASADI_PATCH_VERSION 7
#define CASADI_IS_RELEASE 1
#define CASADI_VERSION_STRING "3.6.7"
#define CASADI_GIT_REVISION ""  // NOLINT(whitespace/line_length)
#define CASADI_GIT_DESCRIBE ""  // NOLINT(whitespace/line_length)
#define CASADI_FEATURE_LIST "\n * dynamic-loading, Support for import of FMI 2.0 binaries\n * sundials-interface, Interface to the ODE/DAE integrator suite SUNDIALS.\n * csparse-interface, Interface to the sparse direct linear solver CSparse.\n * tinyxml-interface, Interface to the XML parser TinyXML.\n * qpoases-interface, Interface to the active-set QP solver qpOASES.\n * lapack-interface, Interface to LAPACK.\n * ipopt-interface, Interface to the NLP solver Ipopt.\n"  // NOLINT(whitespace/line_length)
#define CASADI_BUILD_TYPE "Release"  // NOLINT(whitespace/line_length)
#define CASADI_COMPILER_ID "GNU"  // NOLINT(whitespace/line_length)
#define CASADI_COMPILER "/usr/bin/g++"  // NOLINT(whitespace/line_length)
#define CASADI_COMPILER_FLAGS "  -pipe  -O3 -DNDEBUG -fopenmp -DWITH_OPENMP -pthread -fPIC    -DUSE_CXX11 -DHAVE_MKSTEMPS -DCASADI_WITH_THREAD -DWITH_DEEPBIND -DCASADI_MAJOR_VERSION=3 -DCASADI_MINOR_VERSION=6 -DCASADI_PATCH_VERSION=7 -DCASADI_IS_RELEASE=1 -DCASADI_VERSION=31 -D_USE_MATH_DEFINES -D_SCL_SECURE_NO_WARNINGS -DWITH_DL -DWITH_DEPRECATED_FEATURES -DCASADI_DEFAULT_COMPILER_PLUGIN=shell"  // NOLINT(whitespace/line_length)
#define CASADI_MODULES "casadi;casadi_linsol_lapacklu;casadi_linsol_lapackqr;casadi_sundials_common;casadi_integrator_cvodes;casadi_integrator_idas;casadi_rootfinder_kinsol;casadi_nlpsol_ipopt;casadi_conic_qpoases;casadi_linsol_csparse;casadi_linsol_csparsecholesky;casadi_xmlfile_tinyxml;casadi_conic_nlpsol;casadi_conic_qrqp;casadi_conic_ipqp;casadi_nlpsol_qrsqp;casadi_importer_shell;casadi_integrator_rk;casadi_integrator_collocation;casadi_interpolant_linear;casadi_interpolant_bspline;casadi_linsol_symbolicqr;casadi_linsol_qr;casadi_linsol_ldl;casadi_linsol_tridiag;casadi_linsol_lsqr;casadi_nlpsol_sqpmethod;casadi_nlpsol_feasiblesqpmethod;casadi_nlpsol_scpgen;casadi_rootfinder_newton;casadi_rootfinder_fast_newton;casadi_rootfinder_nlpsol"  // NOLINT(whitespace/line_length)
#define CASADI_PLUGINS "Linsol::lapacklu;Linsol::lapackqr;Integrator::cvodes;Integrator::idas;Rootfinder::kinsol;Nlpsol::ipopt;Conic::qpoases;Linsol::csparse;Linsol::csparsecholesky;XmlFile::tinyxml;Conic::nlpsol;Conic::qrqp;Conic::ipqp;Nlpsol::qrsqp;Importer::shell;Integrator::rk;Integrator::collocation;Interpolant::linear;Interpolant::bspline;Linsol::symbolicqr;Linsol::qr;Linsol::ldl;Linsol::tridiag;Linsol::lsqr;Nlpsol::sqpmethod;Nlpsol::feasiblesqpmethod;Nlpsol::scpgen;Rootfinder::newton;Rootfinder::fast_newton;Rootfinder::nlpsol"  // NOLINT(whitespace/line_length)
#define CASADI_INSTALL_PREFIX "/opt/openrobots"  // NOLINT(whitespace/line_length)
#define CASADI_SHARED_LIBRARY_PREFIX "lib"  // NOLINT(whitespace/line_length)
#define CASADI_SHARED_LIBRARY_SUFFIX ".so"  // NOLINT(whitespace/line_length)
#define CASADI_OBJECT_FILE_SUFFIX ".o"  // NOLINT(whitespace/line_length)

#endif  // CASADI_CONFIG_H // NOLINT(build/header_guard)
