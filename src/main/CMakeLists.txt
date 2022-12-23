# TODO: what does this do?
file(TO_CMAKE_PATH "$ENV{IDF_PATH}" NORMAL_IDF_PATH)

idf_component_register(
	SRCS
	"main.cpp"
	"console.cpp"
	"quadrenc.cpp"
	"raw_imu.cpp"
	"motor.cpp"
	"cobs.cpp"
	"crccalc.cpp"
	"sys_globals.cpp"

	"PS4-esp32/ps4.c"
	"PS4-esp32/ps4_l2cap.c"
	"PS4-esp32/ps4_parser.c"
	"PS4-esp32/ps4_spp.c"
	"PS4-esp32/PS4Controller.cpp"

	INCLUDE_DIRS
	"PS4-esp32"

        PRIV_INCLUDE_DIRS
        ${NORMAL_IDF_PATH}/components/bt/host/bluedroid/stack/include
        ${NORMAL_IDF_PATH}/components/bt/host/bluedroid/common/include
        ${NORMAL_IDF_PATH}/components/bt/common/include
)
