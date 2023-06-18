// ���������� ������������ ���� ���������� p-net
#include <pnet_api.h>

// ���������� ��������� ��� ��������� IO-����������
#define APP_DEFAULT_STATION_NAME "p-net_io_device" // ��� IO-����������
#define APP_API 0 // ����� API
#define APP_SLOT 1 // ����� �����
#define APP_SUBSLOT 1 // ����� ��������
#define APP_INPUT_SIZE 1 // ������ ������� ������ � ������
#define APP_OUTPUT_SIZE 1 // ������ �������� ������ � ������
#define TICK_INTERVAL_US 1000 // �������� ������� � �������������

// ������� ���������� ���������� ��� �������� ��������� p-net
static pnet_t* net = NULL;

// ������� ��� ��������� ������� �� IO-�����������
static int app_pnet_event_handler(
    pnet_t* net,
    void* arg,
    uint32_t arep,
    pnet_event_values_t event)
{
    // ��������� ��� �������
    switch (event)
    {
    case PNET_EVENT_ABORT: // ������� ������ ����������
        printf("Connection aborted\n");
        break;
    case PNET_EVENT_STARTUP: // ������� ������� ����������
        printf("Connection started\n");
        break;
    case PNET_EVENT_PRMEND: // ������� ���������� �������� ����������
        printf("Parameters received\n");
        // ������������ ��������� ����������
        pnet_application_ready(net, arep);
        break;
    case PNET_EVENT_APPLRDY: // ������� ���������� ����������
        printf("Application ready\n");
        break;
    case PNET_EVENT_DATA: // ������� ���������� � ������ �������
        printf("Ready for data exchange\n");
        break;
    default: // ����������� �������
        printf("Unknown event\n");
        break;
    }
    return 0;
}

// ������� ��� ������ ������� �� IO-�����������
static int app_pnet_read_handler(
    pnet_t* net,
    void* arg,
    uint32_t arep,
    uint16_t api,
    uint16_t slot,
    uint16_t subslot,
    uint16_t idx,
    uint8_t* p_read_data,
    uint16_t read_length,
    pnet_result_t* p_result)
{
    // ��������� ����� API, ����� � ��������
    if (api == APP_API && slot == APP_SLOT && subslot == APP_SUBSLOT)
    {
        // ��������� ����� ������� ������
        if (idx == 1234) // ������ ������ ������� ������
        {
            // ��������� ����� ������������� ������
            if (read_length == sizeof(uint32_t)) // ������ ����� ������ � ������
            {
                // ���������� ������ ��� ������ (������ ������)
                uint32_t value = 42; // ������ �������� ������

                // �������� ������ � ����� ��� ������
                memcpy(p_read_data, &value, sizeof(value));

                // ������������� ��������� ������ ��� ��������
                p_result->pnio_status.error_code = PNET_ERROR_CODE_NO_ERROR;
                p_result->pnio_status.error_decode = PNET_ERROR_DECODE_NO_ERROR;
                p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_NO_ERROR;
                p_result->pnio_status.error_code_2 = PNET_ERROR_CODE_2_NO_ERROR;
                p_result->add_data_1 = 0;
                p_result->add_data_2 = 0;

                printf("Read successful\n");
            }
            else // �������� ����� ������������� ������
            {
                // ������������� ��������� ������ ��� ���������
                p_result->pnio_status.error_code = PNET_ERROR_CODE_READ;
                p_result->pnio_status.error_decode = PNET_ERROR_DECODE_PNIO;
                p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_ACC_INVALID_LENGTH;
                p_result->pnio_status.error_code_2 = 0;
                p_result->add_data_1 = 0;
                p_result->add_data_2 = 0;

                printf("Read failed\n");
            }
        }
        else // ����������� ����� ������� ������
        {
            // ������������� ��������� ������ ��� ���������
            p_result->pnio_status.error_code = PNET_ERROR_CODE_READ;
            p_result->pnio_status.error_decode = PNET_ERROR_DECODE_PNIO;
            p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_ACC_INVALID_INDEX;
            p_result->pnio_status.error_code_2 = 0;
            p_result->add_data_1 = 0;
            p_result->add_data_2 = 0;

            printf("Read failed\n");
        }
    }
    else // ����������� ����� API, ����� ��� ��������
    {
        // ������������� ��������� ������ ��� ���������
        p_result->pnio_status.error_code = PNET_ERROR_CODE_READ;
        p_result->pnio_status.error_decode = PNET_ERROR_DECODE_PNIO;
        p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_ACC_INVALID_SLOT_SUBSLOT;
        p_result->pnio_status.error_code_2 = 0;
        p_result->add_data_1 = 0;
        p_result->add_data_2 = 0;

        printf("Read failed\n");
    }
    return 0;
}

// ������� ��� ������ ������� �� IO-�����������
static int app_pnet_write_handler(
    pnet_t* net,
    void* arg,
    uint32_t arep,
    uint16_t api,
    uint16_t slot,
    uint16_t subslot,
    uint16_t idx,
    uint16_t sequence_number,
    uint8_t* p_write_data,
    uint16_t write_length,
    pnet_result_t* p_result)
{
    // ��������� ����� API, ����� � ��������
    if (api == APP_API && slot == APP_SLOT && subslot == APP_SUBSLOT)
    {
        // ��������� ����� ������� ������
        if (idx == 1234) // ������ ������ ������� ������
        {
            // ��������� ����� ������������ ������
            if (write_length == sizeof(uint32_t)) // ������ ����� ������ � ������
            {
                // ������ ������ �� ������ ��� ������
                uint32_t value; // ������ �������� ������
                memcpy(&value, p_write_data, sizeof(value));

                // ������������ ������ ��� ������ (������ ������)
                printf("Received data: %u\n", value);

                // ������������� ��������� ������ ��� ��������
                p_result->pnio_status.error_code = PNET_ERROR_CODE_NO_ERROR;
                p_result->pnio_status.error_decode = PNET_ERROR_DECODE_NO_ERROR;
                p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_NO_ERROR;
                p_result->pnio_status.error_code_2 = PNET_ERROR_CODE_2_NO_ERROR;
                p_result->add_data_1 = 0;
                p_result->add_data_2 = 0;

                printf("Write successful\n");
            }
            else // �������� ����� ������������ ������
            {
                // ������������� ��������� ������ ��� ���������
                p_result->pnio_status.error_code = PNET_ERROR_CODE_WRITE;
                p_result->pnio_status.error_decode = PNET_ERROR_DECODE_PNIO;
                p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_ACC_INVALID_LENGTH;
                p_result->pnio_status.error_code_2 = 0;
                p_result->add_data_1 = 0;
                p_result->add_data_2 = 0;

                printf("Write failed\n");
            }
        }
        else // ����������� ����� ������� ������
        {
            // ������������� ��������� ������ ��� ���������
            p_result->pnio_status.error_code = PNET_ERROR_CODE_WRITE;
            p_result->pnio_status.error_decode = PNET_ERROR_DECODE_PNIO;
            p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_ACC_INVALID_INDEX;
            p_result->pnio_status.error_code_2 = 0;
            p_result->add_data_1 = 0;
            p_result->add_data_2 = 0;

            printf("Write failed\n");
        }
    }
    else // ����������� ����� API, ����� ��� ��������
    {
        // ������������� ��������� ������ ��� ���������
        p_result->pnio_status.error_code = PNET_ERROR_CODE_WRITE;
        p_result->pnio_status.error_decode = PNET_ERROR_DECODE_PNIO;
        p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_ACC_INVALID_SLOT_SUBSLOT;
        p_result->pnio_status.error_code_2 = 0;
        p_result->add_data_1 = 0;
        p_result->add_data_2 = 0;

        printf("Write failed\n");
    }
    return 0;
}

// ������� ��� ������������� ���� profinet
static void app_pnet_init(void)
{
    // ������� ��������� ��� ��������� IO-����������
    pnet_cfg_t cfg;

    // ��������� ��������� ����������� IO-���������� (������ ������)
    memset(&cfg, 0, sizeof(cfg)); // �������� ���������
    cfg.tick_us = TICK_INTERVAL_US; // ������������� �������� �������
    cfg.min_device_interval = 32; // ������������� ����������� �������� ����� ������������
    strcpy(cfg.station_name, APP_DEFAULT_STATION_NAME); // ������������� ��� ����������
    cfg.cb_arg = NULL; // ������������� �������� ��� �������� �������
    cfg.pnal_cfg.handle = NULL; // ������������� ���������� ��� ���� ������������
    cfg.pnal_cfg.arg = NULL; // ������������� �������� ��� ���� ������������
    cfg.state_cb = app_pnet_event_handler; // ������������� ������� ��� ��������� ������� �� IO-�����������
    cfg.connect_cb = NULL; // ������������� ������� ��� ��������� �������� �� ���������� (�� ������������)
    cfg.release_cb = NULL; // ������������� ������� ��� ��������� �������� �� ���������� (�� ������������)
    cfg.dcontrol_cb = NULL; // ������������� ������� ��� ��������� �������� �� ���������� ������� (�� ������������)
    cfg.ccontrol_cb = NULL; // ������������� ������� ��� ��������� �������� �� ���������� ����������� (�� ������������)
    cfg.read_cb = app_pnet_read_handler; // ������������� ������� ��� ��������� �������� �� ������ �������
    cfg.write_cb = app_pnet_write_handler; // ������������� ������� ��� ��������� �������� �� ������ �������
    cfg.exp_module_cb = NULL; // ������������� ������� ��� ��������� �������� �� ����������� ������� (�� ������������)
    cfg.exp_submodule_cb = NULL; // ������������� ������� ��� ��������� �������� �� ����������� ���������� (�� ������������)
    cfg.new_data_status_cb = NULL; // ������������� ������� ��� ��������� ��������� ������� ������ (�� ������������)
    cfg.alarm_ind_cb = NULL; // ������������� ������� ��� ��������� ��������� ������� (�� ������������)
    cfg.alarm_cnf_cb = NULL; // ������������� ������� ��� ��������� ������������� ������� (�� ������������)
    cfg.reset_cb = NULL; // ������������� ������� ��� ��������� �������� �� ����� (�� ������������)

    // �������������� �������� p-net
    net = pnet_init(NULL, &cfg);
    if (net == NULL) // ��������� ���������� �������������
    {
        printf("P-net initialization failed\n");
        exit(EXIT_FAILURE);
    }
    printf("P-net initialized\n");
}

// ������� ��� ��������� IO-����������
static void app_pnet_configure_device(void)
{
    // ������� ��������� ��� �������� ������ � ���������
    pnet_data_cfg_t data_cfg;
    pnet_mod_cfg_t mod_cfg;
    pnet_submod_cfg_t submod_cfg;

    // ��������� ��������� �������� ������ � ��������� (������ ������)
    memset(&data_cfg, 0, sizeof(data_cfg)); // �������� ���������
    data_cfg.data_dir = PNET_DIR_IO; // ������������� ����������� �������� ������
    data_cfg.insize = APP_INPUT_SIZE; // ������������� ������ ������� ������
    data_cfg.outsize = APP_OUTPUT_SIZE; // ������������� ������ �������� ������

    memset(&submod_cfg, 0, sizeof(submod_cfg)); // �������� ���������
    submod_cfg.slot_number = APP_SLOT; // ������������� ����� �����
    submod_cfg.subslot_number = APP_SUBSLOT; // ������������� ����� ��������
    submod_cfg.module_ident_number = 0x00000001; // ������������� ������������� ������
    submod_cfg.submodule_ident_number = 0x00000001; // ������������� ������������� ���������
    submod_cfg.submodule_properties.type = PNET_SUBMOD_TYPE_NO_DATA; // ������������� ��� ���������
    submod_cfg.submodule_properties.shared_input = false; // ������������� ���� ������������ �����
    submod_cfg.submodule_properties.reduce_input_submodule_data_length = false; // ������������� ���� ���������� ����� ������� ������
    submod_cfg.submodule_properties.reduce_output_submodule_data_length = false; // ������������� ���� ���������� ����� �������� ������
    submod_cfg.submodule_properties.discard_ioxs = false; // ������������� ���� ������������ IOXS
    submod_cfg.p_data_cfg = &data_cfg; // ������������� ��������� �� ��������� ������

    memset(&mod_cfg, 0, sizeof(mod_cfg)); // �������� ���������
    mod_cfg.api = APP_API; // ������������� ����� API
    mod_cfg.slot_number = APP_SLOT; // ������������� ����� �����
    mod_cfg.module_ident_number = 0x00000001; // ������������� ������������� ������
    mod_cfg.module_properties.type = PNET_MOD_TYPE_LOGICAL; // ������������� ��� ������

    // ����������� IO-���������� � ������� �������� �������� ������ � ���������
    int ret = pnet_plug_module(net, &mod_cfg); // ���������� ������ � IO-����������
    if (ret != 0) // ��������� ���������� ����������� ������
    {
        printf("Module plug failed\n");
        exit(EXIT_FAILURE);
    }
    printf("Module plugged\n");

    ret = pnet_plug_submodule(net, &submod_cfg); // ���������� ��������� � IO-����������
    if (ret != 0) // ��������� ���������� ����������� ���������
    {
        printf("Submodule plug failed\n");
        exit(EXIT_FAILURE);
    }
    printf("Submodule plugged\n");
}

// ������� ��� ������ ������� � IO-������������
static void app_pnet_handle_data(void)
{
    // ������� ������ ��� �������� ������� � �������� ������
    uint8_t inputdata[APP_INPUT_SIZE]; // ����� ��� ������� ������
    uint8_t outputdata[APP_OUTPUT_SIZE]; // ����� ��� �������� ������

    // ������� ���������� ��� �������� �������� ������
    uint8_t inputdata_iops; // ������ ������� ������ (IOPS)
    uint8_t outputdata_iops; // ������ �������� ������ (IOPS)
    bool inputdata_iocs; // ������ �������� ������ (IOCS)
    bool outputdata_iocs; // ������ ��������� ������ (IOCS)

    // ������ ������� ������ �� IO-�����������
    int ret = pnet_input_get_data_and_iops(
        net,
        APP_API,
        APP_SLOT,
        APP_SUBSLOT,
        &inputdata[0],
        sizeof(inputdata),
        &inputdata_iops);

    if (ret == 0) // ��������� ���������� ������ ������� ������
    {
        printf("Input data received: %u\n", inputdata[0]); // ������� �������� ������ (������ ������)

        ret = pnet_input_get_iocs(
            net,
            APP_API,
            APP_SLOT,
            APP_SUBSLOT,
            &inputdata_iocs);

        if (ret == 0) // ��������� ���������� ������ ������� �������� ������
        {
            printf("Input data status: %u\n", inputdata_iocs); // ������� �������� ������ (������ ������)
        }
        else // ������ ������ ������� �������� ������
        {
            printf("Input data status read failed\n");
        }
    }
    else // ������ ������ ������� ������
    {
        printf("Input data read failed\n");
    }

    // ���������� �������� ������ ��� IO-����������� (������ ������)
    outputdata[0] = inputdata[0] + 1; // ������ �������� ������
    outputdata_iops = PNET_IOXS_GOOD; // ������ ������� �������� ������
    outputdata_iocs = true; // ������ ������� ��������� ������

    // ����� �������� ������ ��� IO-�����������
    ret = pnet_output_set_data_and_iops(
        net,
        APP_API,
        APP_SLOT,
        APP_SUBSLOT,
        &outputdata[0],
        sizeof(outputdata),
        outputdata_iops);

    if (ret == 0) // ��������� ���������� ������ �������� ������
    {
        printf("Output data sent: %u\n", outputdata[0]); // ������� ������������ ������ (������ ������)

        ret = pnet_output_set_iocs(
            net,
            APP_API,
            APP_SLOT,
            APP_SUBSLOT,
            outputdata_iocs);

        if (ret == 0) // ��������� ���������� ������ ������� ��������� ������
        {
            printf("Output data status sent: %u\n", outputdata_iocs); // ������� ������������ ������ (������ ������)
        }
        else // ������ ������ ������� ��������� ������
        {
            printf("Output data status write failed\n");
        }
    }
    else // ������ ������ �������� ������
    {
        printf("Output data write failed\n");
    }
}

// ������� ������� ���������
int main(void)
{
    // �������������� ���� profinet
    app_pnet_init();

    // ����������� IO-����������
    app_pnet_configure_device();

    // ��������� ����������� ���� ��� ������ ������� � IO-������������
    while (true)
    {
        // ��������� ��������� ������� �� IO-�����������
        pnet_handle_periodic(net);

        // ��������� ����� ������� � IO-������������
        app_pnet_handle_data();

        // ���� ���������� ���� �������
        usleep(TICK_INTERVAL_US);
    }

    return 0;
}
// ������� ��� ��������� ������� �� IO-�����������
static void app_pnet_handle_events(void)
{
    // ������� ��������� ��� �������� �������
    pnet_event_values_t event;

    // ��������� ������� ������� �� IO-�����������
    int ret = pnet_get_ar_error_codes(net, &event);

    if (ret == 0) // ��������� ���������� ��������� �������
    {
        // ������������ ������ ���� �������
        switch (event.state)
        {
        case PNET_EVENT_ABORT: // ������� ���������� �����
            printf("Connection aborted\n");
            break;
        case PNET_EVENT_STARTUP: // ������� ������� �����
            printf("Connection started\n");
            break;
        case PNET_EVENT_PRMEND: // ������� ���������� ��������������
            printf("Parameterization done\n");
            break;
        case PNET_EVENT_APPLRDY: // ������� ���������� ����������
            printf("Application ready\n");
            break;
        case PNET_EVENT_DATA: // ������� ������ �������
            printf("Data exchange\n");
            break;
        default: // ����������� �������
            printf("Unknown event\n");
            break;
        }
    }
    else // ������ ��������� �������
    {
        printf("Event read failed\n");
    }
}

// ������� ��� �������� ������ IO-�����������
static void app_pnet_send_alarm(void)
{
    // ������� ��������� ��� �������� ������ ������
    pnet_alarm_argument_t alarm_arg;

    // ��������� ��������� ������� ������
    alarm_arg.api_id = APP_API; // ����� API
    alarm_arg.slot_nbr = APP_SLOT; // ����� �����
    alarm_arg.subslot_nbr = APP_SUBSLOT; // ����� ��������
    alarm_arg.alarm_type = PNET_ALARM_TYPE_PROCESS; // ��� ������ (����������)
    alarm_arg.alarm_specifier = 0x0000; // ������������ ������ (�� ������������)
    alarm_arg.sequence_number = 0x0000; // ���������� ����� ������ (�� ������������)

    // ���������� ����� IO-�����������
    int ret = pnet_alarm_send(net, &alarm_arg);

    if (ret == 0) // ��������� ���������� �������� ������
    {
        printf("Alarm sent\n");
    }
    else // ������ �������� ������
    {
        printf("Alarm send failed\n");
    }
}

// ������� ��� ��������� ������� �� IO-�����������
static void app_pnet_handle_alarms(void)
{
    // ������� ��������� ��� �������� ������ ������ � ������������� ������
    pnet_alarm_argument_t alarm_arg;
    pnet_pnio_status_t alarm_ack;

    // ������ ������ ������ �� IO-�����������
    int ret = pnet_alarm_get(net, &alarm_arg);

    if (ret == 0) // ��������� ���������� ������ ������ ������
    {
        printf("Alarm received\n");

        // ��������� ��������� ������������� ������
        alarm_ack.error_code = PNET_ERROR_CODE_NO_ERROR; // ��� ������ (��� ������)
        alarm_ack.error_decode = PNET_ERROR_DECODE_NO_ERROR; // ������������� ������ (��� ������)
        alarm_ack.error_code_1 = 0x00; // ��� ������ 1 (�� ������������)
        alarm_ack.error_code_2 = 0x00; // ��� ������ 2 (�� ������������)

        // ���������� ������������� ������ IO-�����������
        ret = pnet_alarm_send_ack(net, &alarm_arg, &alarm_ack);

        if (ret == 0) // ��������� ���������� �������� ������������� ������
        {
            printf("Alarm acknowledged\n");
        }
        else // ������ �������� ������������� ������
        {
            printf("Alarm acknowledge failed\n");
        }
    }
    else // ������ ������ ������ ������
    {
        printf("Alarm read failed\n");
    }
}
// ������� ��� ���������� ������ ����� � ������
static void app_pnet_update_io(void)
{
    // ������� ��������� ��� �������� ������ ����� � ������
    uint8_t inputdata[APP_INPUT_DATA_SIZE];
    uint8_t outputdata[APP_OUTPUT_DATA_SIZE];

    // ������ ������ ����� �� IO-�����������
    int ret = pnet_input_get_data_and_iops(net, APP_API, APP_SLOT, APP_SUBSLOT,
        &inputdata[0], sizeof(inputdata),
        &inputdata[APP_INPUT_DATA_SIZE - 1]);

    if (ret == 0) // ��������� ���������� ������ ������ �����
    {
        printf("Input data read\n");

        // ������������ ������ ����� �� ������ ����������
        // ��������, ����� ��������� �������� ������� ����� ������ �����
        if (inputdata[0] == 0x01) // ���� ������ ���� ����� 0x01
        {
            printf("Input data is 0x01\n");

            // ��������� �����-�� ��������
            // ��������, ����� ���������� �������� ������� ����� ������ ������ � 0xFF
            outputdata[0] = 0xFF;
        }
        else // ���� ������ ���� �� ����� 0x01
        {
            printf("Input data is not 0x01\n");

            // ��������� ������ ��������
            // ��������, ����� ���������� �������� ������� ����� ������ ������ � 0x00
            outputdata[0] = 0x00;
        }

        // ������������� ��������� ������ ������ � IOPS � �������
        outputdata[APP_OUTPUT_DATA_SIZE - 1] = PNET_IOXS_GOOD;

        // ���������� ������ ������ � IO-����������
        ret = pnet_output_set_data_and_iops(net, APP_API, APP_SLOT, APP_SUBSLOT,
            &outputdata[0], sizeof(outputdata),
            &outputdata[APP_OUTPUT_DATA_SIZE - 1]);

        if (ret == 0) // ��������� ���������� �������� ������ ������
        {
            printf("Output data sent\n");
        }
        else // ������ �������� ������ ������
        {
            printf("Output data send failed\n");
        }
    }
    else // ������ ������ ������ �����
    {
        printf("Input data read failed\n");
    }
}

// ������� ��� ��������� ��������������� ��������� �� IO-�����������
static void app_pnet_handle_diagnostics(void)
{
    // ������� ��������� ��� �������� ������ ����������� � ������������� �����������
    pnet_diag_source_t diag_source;
    pnet_pnio_status_t diag_status;

    // ������ ������ ����������� �� IO-�����������
    int ret = pnet_diag_get(net, &diag_source);

    if (ret == 0) // ��������� ���������� ������ ������ �����������
    {
        printf("Diagnostic message received\n");

        // ��������� ��������� ������������� �����������
        diag_status.error_code = PNET_ERROR_CODE_NO_ERROR; // ��� ������ (��� ������)
        diag_status.error_decode = PNET_ERROR_DECODE_NO_ERROR; // ������������� ������ (��� ������)
        diag_status.error_code_1 = 0x00; // ��� ������ 1 (�� ������������)
        diag_status.error_code_2 = 0x00; // ��� ������ 2 (�� ������������)

        // ���������� ������������� ����������� IO-�����������
        ret = pnet_diag_send_ack(net, &diag_source, &diag_status);

        if (ret == 0) // ��������� ���������� �������� ������������� �����������
        {
            printf("Diagnostic message acknowledged\n");
        }
        else // ������ �������� ������������� �����������
        {
            printf("Diagnostic message acknowledge failed\n");
        }
    }
    else // ������ ������ ������ �����������
    {
        printf("Diagnostic message read failed\n");
    }
}
// ������� ��� ������ ����� � ����
static void app_pnet_write_log(const char* message)
{
    // ��������� ���� ��� ������ �����
    FILE* logfile = fopen("pnet_log.txt", "a");

    if (logfile != NULL) // ��������� ���������� �������� �����
    {
        printf("Log file opened\n");

        // �������� ������� ����� � ����
        time_t now = time(NULL);
        struct tm* tm_now = localtime(&now);
        char time_str[20];
        strftime(time_str, 20, "%Y-%m-%d %H:%M:%S", tm_now);

        // ���������� ��������� � ���� � ������� fprintf
        fprintf(logfile, "[%s] %s\n", time_str, message);

        // ��������� ����
        fclose(logfile);

        printf("Log file closed\n");
    }
    else // ������ �������� �����
    {
        printf("Log file open failed\n");
    }
}

// ������� ��� ������ ����� �� �����
static void app_pnet_read_log(void)
{
    // ��������� ���� ��� ������ �����
    FILE* logfile = fopen("pnet_log.txt", "r");

    if (logfile != NULL) // ��������� ���������� �������� �����
    {
        printf("Log file opened\n");

        // ������� ����� ��� �������� ��������� �� �����
        char buffer[100];

        // ������ ��������� �� ����� ��������� � ������� fgets
        while (fgets(buffer, 100, logfile) != NULL)
        {
            // ������� ��������� �� ����� � ������� printf
            printf("%s", buffer);
        }

        // ��������� ����
        fclose(logfile);

        printf("Log file closed\n");
    }
    else // ������ �������� �����
    {
        printf("Log file open failed\n");
    }
}
// ������� ��� �������� ���� Profinet
static void app_pnet_close(void)
{
    // �������� ������� pnet_exit()
    int ret = pnet_exit(net);

    if (ret == 0) // ��������� ���������� �������� ����
    {
        printf("Profinet network closed\n");
    }
    else // ������ �������� ����
    {
        printf("Profinet network close failed\n");
    }
}
