#include "radio_data_formats.h"

RadioMessage rx_message = {0};
RadioMessage tx_message = {0};

// stuff below is used to convert uint8_t* data to specific format, depending on message type
RadioDevInfo radiodevinfo = {0};
SensorData sensordata = {0};
uint16_t TmpID = 0;


RM_WhoAmI_e radio_get_whoami()
{
    return ((rx_message.header & RM_Mask_WhoAmI) >> RM_Bit_WhoAmI) ;
}
RM_Dest_e radio_get_dst()
{
    return ((rx_message.header & RM_Mask_Dst) >> RM_Bit_Dst) ;
}
RM_Typ_e radio_get_msgtype()
{
    return ((rx_message.header & RM_Mask_Typ) >> RM_Bit_Typ);
}

//TODO: review below; probably redundant (there is a radio_tx_set_message_header() already)
void radio_set_whoami(RM_WhoAmI_e whoami)
{
    tx_message.header |= (whoami << RM_Bit_WhoAmI);
}
void radio_set_dst(RM_Dest_e dst)
{
    tx_message.header |= (dst << RM_Bit_Dst);
}
void radio_set_msgtype(RM_Typ_e typ)
{
    tx_message.header |= (typ << RM_Bit_Typ);
}

void radio_tx_set_message_header(RM_WhoAmI_e whoami, RM_Dest_e dst, RM_Typ_e typ)
{
    tx_message.header |= (whoami << RM_Bit_WhoAmI) | (dst << RM_Bit_Dst) | (typ << RM_Bit_Typ);
}

void radio_rx_get_tmp_id(uint16_t * tmp_id)
{
    memcpy(tmp_id, &(rx_message.data[0]), sizeof(uint16_t));
}
void radio_tx_set_tmp_id(uint16_t id)
{
    memcpy(&(tx_message.data[0]), &id, sizeof(uint16_t));
}

uint8_t radio_rx_get_id()
{
    return rx_message.data[0];
}
void radio_tx_set_id(uint8_t id)
{
    tx_message.data[0] = id;
}

void radio_rx_get_radiodevinfo(RadioDevInfo * rdinfo)
{
    //memcpy(rdinfo, &(rx_message.data[0]), sizeof(RadioDevInfo));
    // TODO FIXME: read on casting if it truncates redundant stuff - try simple casting like
    rdinfo = (RadioDevInfo*)&(rx_message.data[0]);
}
void radio_tx_set_radiodevinfo(RadioDevInfo* rdinfo)
{
    memcpy(&(tx_message.data[0]), rdinfo, sizeof(RadioDevInfo));
}

void radio_rx_get_sensordata(SensorData * snsrdata)
{
    //memcpy(snsrdata, &(rx_message.data[0]), sizeof(SensorData));
    // TODO FIXME: read on casting if it truncates redundant stuff - try simple casting like
    snsrdata = (SensorData*)&(rx_message.data[0]);
}
void radio_tx_set_sensordata(SensorData * snsrdata)
{
    memcpy(&(tx_message.data[0]), snsrdata, sizeof(SensorData));
}


SensorDataType get_sensordata_type(SensorData * snsrdata)
{
    //TODO checkme
    return snsrdata->datatype;
}

void set_sensordata_type(SensorData *snsrdata, SensorDataType type)
{
    snsrdata->datatype = type;
}

void set_sensor_data_imu(SensorData *snsrdata, IMUData *data)
{
    memcpy(&(snsrdata->data[0]), data, sizeof(IMUData));
}

void set_sensor_data_beamer(SensorData *snsrdata, BeamerData *data)
{
    memcpy(&(snsrdata->data[0]), data, sizeof(BeamerData));
}
