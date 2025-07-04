# Copyright (c) 2025 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_ERPC_WIFI)
  zephyr_library_sources(
    erpc_wifi.c
	erpc_wifi_socket_offload.c
	service/c_wifi_host_to_ra_client.cpp
	service/wifi_host_to_ra_client.cpp
	service/wifi_host_to_ra_interface.cpp

    service/c_wifi_ra_to_host_server.cpp
	service/wifi_ra_to_host_server.cpp
	service/wifi_ra_to_host_interface.cpp
  )

  zephyr_library_sources_ifdef(CONFIG_ERPC_TRANSPORT_UART erpc_wifi_transport_uart.c)
  zephyr_library_sources_ifdef(CONFIG_ERPC_TRANSPORT_SPI erpc_wifi_transport_spi.c)

  zephyr_include_directories(
	.
	service
	${ZEPHYR_BASE}/subsys/net/lib/sockets
  )
endif()
