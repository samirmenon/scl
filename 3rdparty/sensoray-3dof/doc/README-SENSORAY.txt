
Porting the 2600 Middleware
---------------------------

Pre-built windows and linux versions of the middleware are included in the 2600 SDK.
These pre-built libraries are designed to work with 32/64-bit x86 processors.

In some cases the pre-built middleware may not be usable. For example, it may be necessary to
build the middleware for a different operating system, or perhaps rebuild the linux middleware
to work with a big-endian cpu. A complete linux reference design is included in the SDK to help
you in this endeavor. Regardless of your target operating system or cpu type, you will need the
following files to build the middlware library:

s26core.c
  This code encapsulates all of the api functions and the protocols used to communicate with a
  Sensoray 2600 system. It is designed to work with any operating system and any cpu. Public
  functions are exposed directly to application	programs as API calls. When building the middleware,
  you must declare cpu endianness with one of these preprocessor definitions: ENDIAN_LITTLE, ENDIAN_BIG.
  DO NOT MODIFY THIS FILE!

s26mod.c
  Implements the functions required by the core that are specific to your target operating system.
  This is compiled and linked to s26core.c to produce the middleware library. You must create the
  functions if you are porting to a different operating system, whereas no modification is needed
  if you are simply changing the endianness of the linux middleware.

s26mod.h
  Declarations that are shared by the core and mod that are specific to your target operating
  system. This is needed only for building the middleware, and it is already included where needed.
  You must modify this file as required if you are porting to another operating system.

s26modf.h
  Declarations that are shared by the core and mod. This is needed only for building the
  middleware, and it is already included where needed.
  DO NOT MODIFY THIS FILE!

app2600.h
  Declarations that are shared by the core, mod and your application. You must include this in
  any application source code file that calls middleware functions.
  DO NOT MODIFY THIS FILE!

tctable.h
  This file is only included by the core.
  DO NOT MODIFY THIS FILE!

s26app.h
  Declarations that are shared by the core, mod and your application. It is already included where
  needed. You must edit these declarations as necessary for your target operating system and cpu.


In summary:

1. Modify these files as required: s26mod.c, s26mod.h and s26app.h.
2. Build the middleware. A linux shell script, makelib, shows how to do this for linux builds.
3. Good luck! Sensoray has provided you with the foundation code, but Sensoray DOES NOT	PROVIDE
   SUPPORT for customization of the code. We will be happy to quote a fee for porting the
   middleware library to another operating system or cpu.

