################################################################################
# Acquisition Makefile
################################################################################

################################################################################
# Key paths and settings
################################################################################

SDK_PATH = /home/user1/workspace

OPENCV_CFLAGS = `pkg-config --cflags opencv`
OPENCV_LIB = `pkg-config --libs opencv`


CFLAGS += -std=c++11 
CC = g++ ${CFLAGS} ${OPENCV_CFLAGS}

OUTPUTNAME = Acquisition${D}
OUTDIR = ./

################################################################################
# Dependencies
################################################################################
# Spinnaker deps
SPINNAKER_LIB = -L${SDK_PATH}/spinnaker_sdk/lib -lSpinnaker${D}


################################################################################
# Master inc/lib/obj/dep settings
################################################################################
OBJ = Acquisition.o
INC = -I${SDK_PATH}/spinnaker_sdk/include
LIB += -Wl,-Bdynamic ${SPINNAKER_LIB}  ${OPENCV_LIB}
LIB += -Wl,-rpath-link=${SDK_PATH}/spinnaker_sdk/lib 

################################################################################
# Rules/recipes
################################################################################
# Final binary
${OUTPUTNAME}: ${OBJ}
	${CC} -o ${OUTPUTNAME} ${OBJ} ${LIB}

# Intermediate objects
%.o: %.cpp
	${CC} ${CFLAGS} ${INC} -Wall -c -D LINUX $*.cpp

# Clean up intermediate objects
clean_obj:
	rm -f ${OBJ}	@echo "all cleaned up!"

# Clean up everything.
clean:
	rm -f ${OUTDIR}/${OUTPUTNAME} ${OBJ}	@echo "all cleaned up!"
