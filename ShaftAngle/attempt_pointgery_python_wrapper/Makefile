################################################################################
# Acquisition Makefile
################################################################################

################################################################################
# Key paths and settings
################################################################################

SDK_PATH = /home/user1/workspace/robot-runner

################################################################################
# Dependencies
################################################################################

# CFlags

SPINNAKER_CFLAGS += -std=c++11 
OPENCV_CFLAGS = `pkg-config --cflags opencv`

# Libraries
SPINNAKER_LIB = ${SDK_PATH}/spinnaker_sdk/lib -lSpinnaker${D}
OPENCV_LIB = `pkg-config --libs opencv`
PYTHON_LIB = -lpython2.7

# Includes
SPINNAKER_INC = ${SDK_PATH}/spinnaker_sdk/include
PYTHON_INC =/usr/include/python2.7/


################################################################################
# Master settings
################################################################################
OBJ = getImage.o

CC = g++ ${SPINNAKER_CFLAGS} ${OPENCV_CFLAGS}

INC += -I${SPINNAKER_INC} -I${PYTHON_INC}

LIB += -Wl,-Bdynamic -L${SPINNAKER_LIB}  ${OPENCV_LIB} ${PYTHON_LIB}
LIB += -Wl,-rpath-link=${SDK_PATH}/spinnaker_sdk/lib 

OUTPUTNAME = Acquisition${D}
OUTDIR = ./


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
