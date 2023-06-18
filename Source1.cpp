// Подключаем заголовочный файл библиотеки p-net
#include <pnet_api.h>

// Определяем константы для настройки IO-устройства
#define APP_DEFAULT_STATION_NAME "p-net_io_device" // Имя IO-устройства
#define APP_API 0 // Номер API
#define APP_SLOT 1 // Номер слота
#define APP_SUBSLOT 1 // Номер подслота
#define APP_INPUT_SIZE 1 // Размер входных данных в байтах
#define APP_OUTPUT_SIZE 1 // Размер выходных данных в байтах
#define TICK_INTERVAL_US 1000 // Интервал таймера в микросекундах

// Создаем глобальную переменную для хранения контекста p-net
static pnet_t* net = NULL;

// Функция для обработки событий от IO-контроллера
static int app_pnet_event_handler(
    pnet_t* net,
    void* arg,
    uint32_t arep,
    pnet_event_values_t event)
{
    // Проверяем тип события
    switch (event)
    {
    case PNET_EVENT_ABORT: // Событие отмены соединения
        printf("Connection aborted\n");
        break;
    case PNET_EVENT_STARTUP: // Событие запуска соединения
        printf("Connection started\n");
        break;
    case PNET_EVENT_PRMEND: // Событие завершения передачи параметров
        printf("Parameters received\n");
        // Подтверждаем получение параметров
        pnet_application_ready(net, arep);
        break;
    case PNET_EVENT_APPLRDY: // Событие готовности приложения
        printf("Application ready\n");
        break;
    case PNET_EVENT_DATA: // Событие готовности к обмену данными
        printf("Ready for data exchange\n");
        break;
    default: // Неизвестное событие
        printf("Unknown event\n");
        break;
    }
    return 0;
}

// Функция для чтения записей от IO-контроллера
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
    // Проверяем номер API, слота и подслота
    if (api == APP_API && slot == APP_SLOT && subslot == APP_SUBSLOT)
    {
        // Проверяем номер индекса записи
        if (idx == 1234) // Пример номера индекса записи
        {
            // Проверяем длину запрашиваемых данных
            if (read_length == sizeof(uint32_t)) // Пример длины данных в байтах
            {
                // Генерируем данные для чтения (просто пример)
                uint32_t value = 42; // Пример значения данных

                // Копируем данные в буфер для чтения
                memcpy(p_read_data, &value, sizeof(value));

                // Устанавливаем результат чтения как успешный
                p_result->pnio_status.error_code = PNET_ERROR_CODE_NO_ERROR;
                p_result->pnio_status.error_decode = PNET_ERROR_DECODE_NO_ERROR;
                p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_NO_ERROR;
                p_result->pnio_status.error_code_2 = PNET_ERROR_CODE_2_NO_ERROR;
                p_result->add_data_1 = 0;
                p_result->add_data_2 = 0;

                printf("Read successful\n");
            }
            else // Неверная длина запрашиваемых данных
            {
                // Устанавливаем результат чтения как ошибочный
                p_result->pnio_status.error_code = PNET_ERROR_CODE_READ;
                p_result->pnio_status.error_decode = PNET_ERROR_DECODE_PNIO;
                p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_ACC_INVALID_LENGTH;
                p_result->pnio_status.error_code_2 = 0;
                p_result->add_data_1 = 0;
                p_result->add_data_2 = 0;

                printf("Read failed\n");
            }
        }
        else // Неизвестный номер индекса записи
        {
            // Устанавливаем результат чтения как ошибочный
            p_result->pnio_status.error_code = PNET_ERROR_CODE_READ;
            p_result->pnio_status.error_decode = PNET_ERROR_DECODE_PNIO;
            p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_ACC_INVALID_INDEX;
            p_result->pnio_status.error_code_2 = 0;
            p_result->add_data_1 = 0;
            p_result->add_data_2 = 0;

            printf("Read failed\n");
        }
    }
    else // Неизвестный номер API, слота или подслота
    {
        // Устанавливаем результат чтения как ошибочный
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

// Функция для записи записей от IO-контроллера
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
    // Проверяем номер API, слота и подслота
    if (api == APP_API && slot == APP_SLOT && subslot == APP_SUBSLOT)
    {
        // Проверяем номер индекса записи
        if (idx == 1234) // Пример номера индекса записи
        {
            // Проверяем длину записываемых данных
            if (write_length == sizeof(uint32_t)) // Пример длины данных в байтах
            {
                // Читаем данные из буфера для записи
                uint32_t value; // Пример значения данных
                memcpy(&value, p_write_data, sizeof(value));

                // Обрабатываем данные для записи (просто пример)
                printf("Received data: %u\n", value);

                // Устанавливаем результат записи как успешный
                p_result->pnio_status.error_code = PNET_ERROR_CODE_NO_ERROR;
                p_result->pnio_status.error_decode = PNET_ERROR_DECODE_NO_ERROR;
                p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_NO_ERROR;
                p_result->pnio_status.error_code_2 = PNET_ERROR_CODE_2_NO_ERROR;
                p_result->add_data_1 = 0;
                p_result->add_data_2 = 0;

                printf("Write successful\n");
            }
            else // Неверная длина записываемых данных
            {
                // Устанавливаем результат записи как ошибочный
                p_result->pnio_status.error_code = PNET_ERROR_CODE_WRITE;
                p_result->pnio_status.error_decode = PNET_ERROR_DECODE_PNIO;
                p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_ACC_INVALID_LENGTH;
                p_result->pnio_status.error_code_2 = 0;
                p_result->add_data_1 = 0;
                p_result->add_data_2 = 0;

                printf("Write failed\n");
            }
        }
        else // Неизвестный номер индекса записи
        {
            // Устанавливаем результат записи как ошибочный
            p_result->pnio_status.error_code = PNET_ERROR_CODE_WRITE;
            p_result->pnio_status.error_decode = PNET_ERROR_DECODE_PNIO;
            p_result->pnio_status.error_code_1 = PNET_ERROR_CODE_1_ACC_INVALID_INDEX;
            p_result->pnio_status.error_code_2 = 0;
            p_result->add_data_1 = 0;
            p_result->add_data_2 = 0;

            printf("Write failed\n");
        }
    }
    else // Неизвестный номер API, слота или подслота
    {
        // Устанавливаем результат записи как ошибочный
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

// Функция для инициализации сети profinet
static void app_pnet_init(void)
{
    // Создаем структуру для настройки IO-устройства
    pnet_cfg_t cfg;

    // Заполняем структуру настройками IO-устройства (просто пример)
    memset(&cfg, 0, sizeof(cfg)); // Обнуляем структуру
    cfg.tick_us = TICK_INTERVAL_US; // Устанавливаем интервал таймера
    cfg.min_device_interval = 32; // Устанавливаем минимальный интервал между устройствами
    strcpy(cfg.station_name, APP_DEFAULT_STATION_NAME); // Устанавливаем имя устройства
    cfg.cb_arg = NULL; // Устанавливаем аргумент для обратных вызовов
    cfg.pnal_cfg.handle = NULL; // Устанавливаем дескриптор для слоя портирования
    cfg.pnal_cfg.arg = NULL; // Устанавливаем аргумент для слоя портирования
    cfg.state_cb = app_pnet_event_handler; // Устанавливаем функцию для обработки событий от IO-контроллера
    cfg.connect_cb = NULL; // Устанавливаем функцию для обработки запросов на соединение (не используется)
    cfg.release_cb = NULL; // Устанавливаем функцию для обработки запросов на отключение (не используется)
    cfg.dcontrol_cb = NULL; // Устанавливаем функцию для обработки запросов на управление данными (не используется)
    cfg.ccontrol_cb = NULL; // Устанавливаем функцию для обработки запросов на управление соединением (не используется)
    cfg.read_cb = app_pnet_read_handler; // Устанавливаем функцию для обработки запросов на чтение записей
    cfg.write_cb = app_pnet_write_handler; // Устанавливаем функцию для обработки запросов на запись записей
    cfg.exp_module_cb = NULL; // Устанавливаем функцию для обработки запросов на подключение модулей (не используется)
    cfg.exp_submodule_cb = NULL; // Устанавливаем функцию для обработки запросов на подключение подмодулей (не используется)
    cfg.new_data_status_cb = NULL; // Устанавливаем функцию для обработки изменения статуса данных (не используется)
    cfg.alarm_ind_cb = NULL; // Устанавливаем функцию для обработки индикации алармов (не используется)
    cfg.alarm_cnf_cb = NULL; // Устанавливаем функцию для обработки подтверждения алармов (не используется)
    cfg.reset_cb = NULL; // Устанавливаем функцию для обработки запросов на сброс (не используется)

    // Инициализируем контекст p-net
    net = pnet_init(NULL, &cfg);
    if (net == NULL) // Проверяем успешность инициализации
    {
        printf("P-net initialization failed\n");
        exit(EXIT_FAILURE);
    }
    printf("P-net initialized\n");
}

// Функция для настройки IO-устройства
static void app_pnet_configure_device(void)
{
    // Создаем структуры для описания модуля и подмодуля
    pnet_data_cfg_t data_cfg;
    pnet_mod_cfg_t mod_cfg;
    pnet_submod_cfg_t submod_cfg;

    // Заполняем структуры описания модуля и подмодуля (просто пример)
    memset(&data_cfg, 0, sizeof(data_cfg)); // Обнуляем структуру
    data_cfg.data_dir = PNET_DIR_IO; // Устанавливаем направление передачи данных
    data_cfg.insize = APP_INPUT_SIZE; // Устанавливаем размер входных данных
    data_cfg.outsize = APP_OUTPUT_SIZE; // Устанавливаем размер выходных данных

    memset(&submod_cfg, 0, sizeof(submod_cfg)); // Обнуляем структуру
    submod_cfg.slot_number = APP_SLOT; // Устанавливаем номер слота
    submod_cfg.subslot_number = APP_SUBSLOT; // Устанавливаем номер подслота
    submod_cfg.module_ident_number = 0x00000001; // Устанавливаем идентификатор модуля
    submod_cfg.submodule_ident_number = 0x00000001; // Устанавливаем идентификатор подмодуля
    submod_cfg.submodule_properties.type = PNET_SUBMOD_TYPE_NO_DATA; // Устанавливаем тип подмодуля
    submod_cfg.submodule_properties.shared_input = false; // Устанавливаем флаг разделяемого ввода
    submod_cfg.submodule_properties.reduce_input_submodule_data_length = false; // Устанавливаем флаг сокращения длины входных данных
    submod_cfg.submodule_properties.reduce_output_submodule_data_length = false; // Устанавливаем флаг сокращения длины выходных данных
    submod_cfg.submodule_properties.discard_ioxs = false; // Устанавливаем флаг отбрасывания IOXS
    submod_cfg.p_data_cfg = &data_cfg; // Устанавливаем указатель на структуру данных

    memset(&mod_cfg, 0, sizeof(mod_cfg)); // Обнуляем структуру
    mod_cfg.api = APP_API; // Устанавливаем номер API
    mod_cfg.slot_number = APP_SLOT; // Устанавливаем номер слота
    mod_cfg.module_ident_number = 0x00000001; // Устанавливаем идентификатор модуля
    mod_cfg.module_properties.type = PNET_MOD_TYPE_LOGICAL; // Устанавливаем тип модуля

    // Настраиваем IO-устройство с помощью структур описания модуля и подмодуля
    int ret = pnet_plug_module(net, &mod_cfg); // Подключаем модуль к IO-устройству
    if (ret != 0) // Проверяем успешность подключения модуля
    {
        printf("Module plug failed\n");
        exit(EXIT_FAILURE);
    }
    printf("Module plugged\n");

    ret = pnet_plug_submodule(net, &submod_cfg); // Подключаем подмодуль к IO-устройству
    if (ret != 0) // Проверяем успешность подключения подмодуля
    {
        printf("Submodule plug failed\n");
        exit(EXIT_FAILURE);
    }
    printf("Submodule plugged\n");
}

// Функция для обмена данными с IO-контроллером
static void app_pnet_handle_data(void)
{
    // Создаем буферы для хранения входных и выходных данных
    uint8_t inputdata[APP_INPUT_SIZE]; // Буфер для входных данных
    uint8_t outputdata[APP_OUTPUT_SIZE]; // Буфер для выходных данных

    // Создаем переменные для хранения статусов данных
    uint8_t inputdata_iops; // Статус входных данных (IOPS)
    uint8_t outputdata_iops; // Статус выходных данных (IOPS)
    bool inputdata_iocs; // Статус входного канала (IOCS)
    bool outputdata_iocs; // Статус выходного канала (IOCS)

    // Читаем входные данные от IO-контроллера
    int ret = pnet_input_get_data_and_iops(
        net,
        APP_API,
        APP_SLOT,
        APP_SUBSLOT,
        &inputdata[0],
        sizeof(inputdata),
        &inputdata_iops);

    if (ret == 0) // Проверяем успешность чтения входных данных
    {
        printf("Input data received: %u\n", inputdata[0]); // Выводим принятые данные (просто пример)

        ret = pnet_input_get_iocs(
            net,
            APP_API,
            APP_SLOT,
            APP_SUBSLOT,
            &inputdata_iocs);

        if (ret == 0) // Проверяем успешность чтения статуса входного канала
        {
            printf("Input data status: %u\n", inputdata_iocs); // Выводим принятый статус (просто пример)
        }
        else // Ошибка чтения статуса входного канала
        {
            printf("Input data status read failed\n");
        }
    }
    else // Ошибка чтения входных данных
    {
        printf("Input data read failed\n");
    }

    // Генерируем выходные данные для IO-контроллера (просто пример)
    outputdata[0] = inputdata[0] + 1; // Пример выходных данных
    outputdata_iops = PNET_IOXS_GOOD; // Пример статуса выходных данных
    outputdata_iocs = true; // Пример статуса выходного канала

    // Пишем выходные данные для IO-контроллера
    ret = pnet_output_set_data_and_iops(
        net,
        APP_API,
        APP_SLOT,
        APP_SUBSLOT,
        &outputdata[0],
        sizeof(outputdata),
        outputdata_iops);

    if (ret == 0) // Проверяем успешность записи выходных данных
    {
        printf("Output data sent: %u\n", outputdata[0]); // Выводим отправленные данные (просто пример)

        ret = pnet_output_set_iocs(
            net,
            APP_API,
            APP_SLOT,
            APP_SUBSLOT,
            outputdata_iocs);

        if (ret == 0) // Проверяем успешность записи статуса выходного канала
        {
            printf("Output data status sent: %u\n", outputdata_iocs); // Выводим отправленный статус (просто пример)
        }
        else // Ошибка записи статуса выходного канала
        {
            printf("Output data status write failed\n");
        }
    }
    else // Ошибка записи выходных данных
    {
        printf("Output data write failed\n");
    }
}

// Главная функция программы
int main(void)
{
    // Инициализируем сеть profinet
    app_pnet_init();

    // Настраиваем IO-устройство
    app_pnet_configure_device();

    // Запускаем бесконечный цикл для обмена данными с IO-контроллером
    while (true)
    {
        // Выполняем обработку событий от IO-контроллера
        pnet_handle_periodic(net);

        // Выполняем обмен данными с IO-контроллером
        app_pnet_handle_data();

        // Ждем следующего тика таймера
        usleep(TICK_INTERVAL_US);
    }

    return 0;
}
// Функция для обработки событий от IO-контроллера
static void app_pnet_handle_events(void)
{
    // Создаем структуру для хранения событий
    pnet_event_values_t event;

    // Проверяем наличие событий от IO-контроллера
    int ret = pnet_get_ar_error_codes(net, &event);

    if (ret == 0) // Проверяем успешность получения событий
    {
        // Обрабатываем разные типы событий
        switch (event.state)
        {
        case PNET_EVENT_ABORT: // Событие прерывания связи
            printf("Connection aborted\n");
            break;
        case PNET_EVENT_STARTUP: // Событие запуска связи
            printf("Connection started\n");
            break;
        case PNET_EVENT_PRMEND: // Событие завершения параметризации
            printf("Parameterization done\n");
            break;
        case PNET_EVENT_APPLRDY: // Событие готовности приложения
            printf("Application ready\n");
            break;
        case PNET_EVENT_DATA: // Событие обмена данными
            printf("Data exchange\n");
            break;
        default: // Неизвестное событие
            printf("Unknown event\n");
            break;
        }
    }
    else // Ошибка получения событий
    {
        printf("Event read failed\n");
    }
}

// Функция для отправки аларма IO-контроллеру
static void app_pnet_send_alarm(void)
{
    // Создаем структуру для хранения данных аларма
    pnet_alarm_argument_t alarm_arg;

    // Заполняем структуру данными аларма
    alarm_arg.api_id = APP_API; // Номер API
    alarm_arg.slot_nbr = APP_SLOT; // Номер слота
    alarm_arg.subslot_nbr = APP_SUBSLOT; // Номер подслота
    alarm_arg.alarm_type = PNET_ALARM_TYPE_PROCESS; // Тип аларма (процессный)
    alarm_arg.alarm_specifier = 0x0000; // Спецификатор аларма (не используется)
    alarm_arg.sequence_number = 0x0000; // Порядковый номер аларма (не используется)

    // Отправляем аларм IO-контроллеру
    int ret = pnet_alarm_send(net, &alarm_arg);

    if (ret == 0) // Проверяем успешность отправки аларма
    {
        printf("Alarm sent\n");
    }
    else // Ошибка отправки аларма
    {
        printf("Alarm send failed\n");
    }
}

// Функция для обработки алармов от IO-контроллера
static void app_pnet_handle_alarms(void)
{
    // Создаем структуры для хранения данных аларма и подтверждения аларма
    pnet_alarm_argument_t alarm_arg;
    pnet_pnio_status_t alarm_ack;

    // Читаем данные аларма от IO-контроллера
    int ret = pnet_alarm_get(net, &alarm_arg);

    if (ret == 0) // Проверяем успешность чтения данных аларма
    {
        printf("Alarm received\n");

        // Заполняем структуру подтверждения аларма
        alarm_ack.error_code = PNET_ERROR_CODE_NO_ERROR; // Код ошибки (нет ошибки)
        alarm_ack.error_decode = PNET_ERROR_DECODE_NO_ERROR; // Декодирование ошибки (нет ошибки)
        alarm_ack.error_code_1 = 0x00; // Код ошибки 1 (не используется)
        alarm_ack.error_code_2 = 0x00; // Код ошибки 2 (не используется)

        // Отправляем подтверждение аларма IO-контроллеру
        ret = pnet_alarm_send_ack(net, &alarm_arg, &alarm_ack);

        if (ret == 0) // Проверяем успешность отправки подтверждения аларма
        {
            printf("Alarm acknowledged\n");
        }
        else // Ошибка отправки подтверждения аларма
        {
            printf("Alarm acknowledge failed\n");
        }
    }
    else // Ошибка чтения данных аларма
    {
        printf("Alarm read failed\n");
    }
}
// Функция для обновления данных ввода и вывода
static void app_pnet_update_io(void)
{
    // Создаем структуры для хранения данных ввода и вывода
    uint8_t inputdata[APP_INPUT_DATA_SIZE];
    uint8_t outputdata[APP_OUTPUT_DATA_SIZE];

    // Читаем данные ввода из IO-контроллера
    int ret = pnet_input_get_data_and_iops(net, APP_API, APP_SLOT, APP_SUBSLOT,
        &inputdata[0], sizeof(inputdata),
        &inputdata[APP_INPUT_DATA_SIZE - 1]);

    if (ret == 0) // Проверяем успешность чтения данных ввода
    {
        printf("Input data read\n");

        // Обрабатываем данные ввода по своему усмотрению
        // Например, можно проверить значение первого байта данных ввода
        if (inputdata[0] == 0x01) // Если первый байт равен 0x01
        {
            printf("Input data is 0x01\n");

            // Выполняем какое-то действие
            // Например, можно установить значение первого байта данных вывода в 0xFF
            outputdata[0] = 0xFF;
        }
        else // Если первый байт не равен 0x01
        {
            printf("Input data is not 0x01\n");

            // Выполняем другое действие
            // Например, можно установить значение первого байта данных вывода в 0x00
            outputdata[0] = 0x00;
        }

        // Устанавливаем состояние данных вывода и IOPS в хорошее
        outputdata[APP_OUTPUT_DATA_SIZE - 1] = PNET_IOXS_GOOD;

        // Отправляем данные вывода в IO-контроллер
        ret = pnet_output_set_data_and_iops(net, APP_API, APP_SLOT, APP_SUBSLOT,
            &outputdata[0], sizeof(outputdata),
            &outputdata[APP_OUTPUT_DATA_SIZE - 1]);

        if (ret == 0) // Проверяем успешность отправки данных вывода
        {
            printf("Output data sent\n");
        }
        else // Ошибка отправки данных вывода
        {
            printf("Output data send failed\n");
        }
    }
    else // Ошибка чтения данных ввода
    {
        printf("Input data read failed\n");
    }
}

// Функция для обработки диагностических сообщений от IO-контроллера
static void app_pnet_handle_diagnostics(void)
{
    // Создаем структуры для хранения данных диагностики и подтверждения диагностики
    pnet_diag_source_t diag_source;
    pnet_pnio_status_t diag_status;

    // Читаем данные диагностики от IO-контроллера
    int ret = pnet_diag_get(net, &diag_source);

    if (ret == 0) // Проверяем успешность чтения данных диагностики
    {
        printf("Diagnostic message received\n");

        // Заполняем структуру подтверждения диагностики
        diag_status.error_code = PNET_ERROR_CODE_NO_ERROR; // Код ошибки (нет ошибки)
        diag_status.error_decode = PNET_ERROR_DECODE_NO_ERROR; // Декодирование ошибки (нет ошибки)
        diag_status.error_code_1 = 0x00; // Код ошибки 1 (не используется)
        diag_status.error_code_2 = 0x00; // Код ошибки 2 (не используется)

        // Отправляем подтверждение диагностики IO-контроллеру
        ret = pnet_diag_send_ack(net, &diag_source, &diag_status);

        if (ret == 0) // Проверяем успешность отправки подтверждения диагностики
        {
            printf("Diagnostic message acknowledged\n");
        }
        else // Ошибка отправки подтверждения диагностики
        {
            printf("Diagnostic message acknowledge failed\n");
        }
    }
    else // Ошибка чтения данных диагностики
    {
        printf("Diagnostic message read failed\n");
    }
}
// Функция для записи логов в файл
static void app_pnet_write_log(const char* message)
{
    // Открываем файл для записи логов
    FILE* logfile = fopen("pnet_log.txt", "a");

    if (logfile != NULL) // Проверяем успешность открытия файла
    {
        printf("Log file opened\n");

        // Получаем текущее время и дату
        time_t now = time(NULL);
        struct tm* tm_now = localtime(&now);
        char time_str[20];
        strftime(time_str, 20, "%Y-%m-%d %H:%M:%S", tm_now);

        // Записываем сообщение в файл с помощью fprintf
        fprintf(logfile, "[%s] %s\n", time_str, message);

        // Закрываем файл
        fclose(logfile);

        printf("Log file closed\n");
    }
    else // Ошибка открытия файла
    {
        printf("Log file open failed\n");
    }
}

// Функция для чтения логов из файла
static void app_pnet_read_log(void)
{
    // Открываем файл для чтения логов
    FILE* logfile = fopen("pnet_log.txt", "r");

    if (logfile != NULL) // Проверяем успешность открытия файла
    {
        printf("Log file opened\n");

        // Создаем буфер для хранения сообщений из файла
        char buffer[100];

        // Читаем сообщения из файла построчно с помощью fgets
        while (fgets(buffer, 100, logfile) != NULL)
        {
            // Выводим сообщение на экран с помощью printf
            printf("%s", buffer);
        }

        // Закрываем файл
        fclose(logfile);

        printf("Log file closed\n");
    }
    else // Ошибка открытия файла
    {
        printf("Log file open failed\n");
    }
}
// Функция для закрытия сети Profinet
static void app_pnet_close(void)
{
    // Вызываем функцию pnet_exit()
    int ret = pnet_exit(net);

    if (ret == 0) // Проверяем успешность закрытия сети
    {
        printf("Profinet network closed\n");
    }
    else // Ошибка закрытия сети
    {
        printf("Profinet network close failed\n");
    }
}
