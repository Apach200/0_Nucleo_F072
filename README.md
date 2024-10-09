# 0_Nucleo_F072
CANOpen_Device_at_Nucleo_F072

Проект выполнен в среде STM32CubeIDE


Реализовано CAN-устройство на STM32F072 (EVB Nucleo-F072RB)
Работает в паре с устройством из проекта 0_CAN_F103.
CAN-устройство на STM32F072 считывает запись с индексом 0x6001 из OD CAN-устройства 0_CAN_F103 посредством SDO.
1. Чтение с удалённого узла записи 0x6001
2. Новое значение в  0x6001
3. Чтение с удалённого узла записи 0x6001

OD отредактирован с помощью EDSEditorGUI
Для просмотра открыть DS301_profile.xpd
Перетащить из проводника файл DS301_profile.xdd (CANopen XML Device Description File )