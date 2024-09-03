/**
 * @file EthercatMaster.cpp
 * @brief Implementation of the EthercatMaster class.
 *
 * This file contains the implementation of the EthercatMaster class, which is responsible for
 * initializing and configuring EtherCAT communication, finding and configuring slaves, and
 * synchronizing system time with bus DC time.
 */

#include "EthercatMaster.h"
#include <chrono>
#include <thread>

#include <cstring>
#include <unistd.h>

// #include "Logger.h"

uint8_t EthercatMaster::IOmap[ETHERCAT_SLAVES_IO_MAPS_BUFFER_SIZE];
uint8_t EthercatMaster::currentSlavesWorkingCounter = 0;
uint8_t EthercatMaster::expectedslavesWorkingCounter = 0;
int64_t EthercatMaster::timeDeltaSystemToDcClock = 0;
bool EthercatMaster::allSlavesAreInOperationalMode = 0;
bool EthercatMaster::m_bEcatInitialisedAndMapped = false;
EthercatMaster::ETHERCAT_SLAVE_TYPE EthercatMaster::slaveTypes[];

pthread_mutex_t EthercatMaster::process_data_mutex;

EthercatMaster::EthercatMaster() { memset(slaveTypes, ETHERCAT_SLAVE_INVALID, sizeof(slaveTypes)); }

/**
 * @brief Initialises the EtherCAT communication.
 *
 * This function initialises the EtherCAT communication by binding the socket to the NIC name.
 *
 * @param[in] nic_name The name of the NIC to bind the socket to.
 * @return True if the EtherCAT communication was successfully initialised, false otherwise.
 */
bool EthercatMaster::InitialiseComms(const char* const nic_name)
{
    /* initialise SOEM, bind socket to nic_name */
    return ec_init(const_cast<char*>(nic_name));
}

/**
 * @brief Finds number of slaves on the network.
 *
 * This function finds the slaves by auto-configuring those that support SII, those that don't need to manually
 * configured
 *
 * @return The number of slaves found and configured.
 */
uint8_t EthercatMaster::FindSlavesNumber()
{
    int32_t slave_count = 0;
    // CLogger& refLogger = GetLogger();

    /* find slaves - auto configure those that support SII, manually configure those that don't */
    slave_count = ec_config_init(FALSE);

    if (slave_count > 0)
    {
        // refLogger.LogAndPrint(CLogger::Info, "%d slaves found and configured", ec_slavecount);

        // Wait for all slaves to reach SAFE_OP state
        ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    }
    // else
    // {
    // 	refLogger.LogAndPrint(CLogger::Debug, "Error: EthercatMaster::FindAndConfigureSlaves() - No slaves found or
    // configured.");
    // }

    return slave_count;
}

/**
 * @brief Configures the slave.
 *
 * This function configures the slaves manually configuring
 * those that don't support SII.
 * @param[in] slave_id The ID of the slave to configure.
 * @param[in] ecat_manufacturer_id The ecat manufacturer ID of the slave from EEprom
 * @param[in] ecat_id The ecat ID of the slave from EEprom
 * @param[in] reg_config_func The registered configuration function to link slave specific setup to preop->safeop hook
 * @param[in] slave_type The type of the slave, enum ETHERCAT_SLAVE_TYPE
 * @return 0 if the configuration succeeds, otherwise returns an error code.
 */
uint8_t EthercatMaster::ConfigureSlave(
    uint8_t slave_id, uint32 ecat_manufacturer_id, uint32 ecat_id, int (*reg_config_func)(uint16),
    ETHERCAT_SLAVE_TYPE slave_type
)
{
    if (slave_id > ec_slavecount)
    {
        // refLogger.LogAndPrint(CLogger::Error, "Slave ID %d is out of range", slave_id);
        return -1;
    }

    //    refLogger.LogAndPrint(CLogger::Info, "Slave:%d Name:%s", slave_index, ec_slave[slave_index].name);

    // ZeroErr drive
    if ((ec_slave[slave_id].eep_man == ecat_manufacturer_id) && (ec_slave[slave_id].eep_id == ecat_id))
    {
        //    refLogger.LogAndPrint(CLogger::Debug, "Found %s at position %d", ec_slave[slave_index].name,
        //    slave_index); refLogger.LogAndPrint(CLogger::Debug, "%s doesn't support SII so configure
        //    manually", ec_slave[slave_index].name);
        // link slave specific setup to preop->safeop hook
        ec_slave[slave_id].PO2SOconfig = reg_config_func;

        // slave_index is offset by one in SOEM as slave[0] is reserved for the master node
        slaveTypes[slave_id] = slave_type;
        return 0;
    }
    else
    {
        //    refLogger.LogAndPrint(CLogger::Error, "Unsupported EtherCAT slave found %s at position %d",
        //    ec_slave[slave_index].name, slave_index);
        return 1;
    }
}

/**
 * @brief Configures slaves' distributed clock (DC).
 *
 * This function sets up the distributed clock (DC) synchronization for all connected
 * EtherCAT slaves. It configures the DC options for every DC capable slave found in the list.
 *
 * @param[in] ecat_bus_interval_ms The interval at which the EtherCAT bus operates, in milliseconds.
 * @return 0 if the configuration succeeds, otherwise returns an error code.
 */
uint8_t EthercatMaster::ConfigureSlavesDC(uint32_t ecat_bus_interval_ms)
{
    // Configure DC options for every DC capable slave found in the list
    if (!ec_configdc())
    {
        //    refLogger.LogAndPrint(CLogger::Error, "EthercatMaster::FindAndConfigureSlaves : ec_configdc() failed
        //    to find slaves with DC");
        return -1;
    }

    // Read individual slave state and store in ec_slave[]
    int32_t slave_state = ec_readstate();
    //    refLogger.LogAndPrint(CLogger::Info, "EthercatMaster::FindAndConfigureSlaves : ec_readstate() returns
    //    lowest slave state of %d", slave_state);

    // Tell first DC enabled slave to activate SYNC0 object at ecat_bus_interval_ms
    for (uint8_t slave_index = 1; slave_index <= ec_slavecount; slave_index++)
    {
        if (ec_slave[slave_index].hasdc)
        {
            // refLogger.LogAndPrint(CLogger::Debug, "EthercatMaster::FindAndConfigureSlaves(): activating SYNC0 on
            // slave %d", slave_index);

            ec_dcsync0(slave_index, TRUE, 1000U * ecat_bus_interval_ms, 1000U * ecat_bus_interval_ms / 2);

            // DO WE NEED DC 1?
            // ec_dcsync01(slave_index, TRUE, 1000U * ecat_bus_interval_ms, 0, 1000U * ecat_bus_interval_ms / 2);
            // break;
        }
    }

    if (ec_slave[0].hasdc)
    {
        // refLogger.LogAndPrint(CLogger::Debug, "slave[0] has DC configured");
        return 0;
    }
    else
    {
        // refLogger.LogAndPrint(CLogger::Debug, "slave[0] does NOT have DC configured");
        return 1;
    }
}

/**
 * @brief Configures the slaves' I/O map.
 *
 * This function configures the slaves' I/O map by calling the ec_config_map() function and initialises PDO data mutex.
 *
 * @return True if the configuration succeeds, false otherwise.
 */
bool EthercatMaster::ConfigureSlavesIOMap()
{
    if (ec_config_map(reinterpret_cast<char*>(IOmap)) <= 0)
    {
        // refLogger.LogAndPrint(CLogger::Error, "EthercatMaster::FindAndConfigureSlaves : ec_config_map() failed");
        return false;
    }

    // for (uint8_t slave_index = 1; slave_index <= ec_slavecount ; slave_index++)
    // {
    //     refLogger.LogAndPrint(CLogger::Info, "Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d
    //     delay:%d.%d",
    //             slave_index, ec_slave[slave_index].name, ec_slave[slave_index].Obits,
    //             ec_slave[slave_index].Ibits, ec_slave[slave_index].state,
    //             (int)ec_slave[slave_index].pdelay, ec_slave[slave_index].hasdc);

    //     refLogger.LogAndPrint(CLogger::Info, "        Out:%p,%4d In:%p,%4d",
    //             ec_slave[slave_index].outputs, ec_slave[slave_index].Obytes,
    //             ec_slave[slave_index].inputs, ec_slave[slave_index].Ibytes);
    // }

    // initialise PDO data mutex
    process_data_mutex = PTHREAD_MUTEX_INITIALIZER;

    return true;
}

/**
 * @brief Checks if a slave is ready.
 *
 * This function checks if a slave is ready by checking if the slave is in the operational state.
 *
 * @param[in] slave_id The ID of the slave to check.
 * @return True if the slave is ready, false otherwise.
 */
bool EthercatMaster::IsSlaveReady(uint8_t slave_id)
{
    bool slave_is_ready = false;

    if (slave_id <= ec_slavecount)
    {
        if (ec_slave[slave_id].state == EC_STATE_OPERATIONAL)
        {
            slave_is_ready = true;
        }
    }

    return slave_is_ready;
}

/**
 * @brief Terminates the EtherCAT communication.
 *
 * This function terminates the EtherCAT communication by closing the socket.
 */
void EthercatMaster::TerminateComms()
{
    /* request SAFE_OP state for all slaves */
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);

    ec_close();

    // pthread_mutex_destroy(&process_data_mutex);
}

/**
 * @brief Synchronizes the system time with the bus DC time.
 *
 * This function synchronizes the system time with the bus DC time by calculating the offset time.
 *
 * @param[in] reference_time The reference time.
 * @param[in] cycle_time The cycle time.
 * @param[out] offset_time The offset time.
 */
void EthercatMaster::SyncSystemTimeToBusDcTime(int64_t reference_time, int64_t cycle_time, int64_t* offset_time)
{
    static int64_t integral = 0;
    static int64_t previous_ref_time = 0;
    static int32 wrapped_count = 0;
    static int64_t deltas[100];
    static uint8 first_time = 1;
    static int64 min_dt = 0, max_dt = 0;
    int64_t ref_time_64 = 0;
    int64_t delta;
    uint16 i = 0;
    uint8 found_dt_value_outside_threshold = 0;

    // CLogger& refLogger = GetLogger();

    // Initialise static array
    if (first_time)
    {
        for (i = 0; i < 100; i++)
        {
            deltas[i] = ETHERCAT_DC_CLOCK_STABLE_THRESHOLD_NS + 1;
        }

        first_time = 0;
    }

    // Check if we have wrapped around 32 bits. This is due to the fact that some EtherCAT controllers
    // run a 32-bit DC timer which wraps around every ~4.2s
    if (reference_time < previous_ref_time)
    {
        // Wrap-around detected, calculate interval since last loop
        ref_time_64 = static_cast<int64_t>(0xFFFFFFFFLL - previous_ref_time + reference_time);

        // Increment 32-bit wrap counter
        wrapped_count++;

        // refLogger.LogAndPrint(CLogger::Debug, "\n\n\n\n\n\n\nJust corrected a 32bit overwrap\n\n\n\n\n\n\n\n\n");
    }
    else
    {
        ref_time_64 = reference_time;
    }

    // Add 32-bit wrap-around to absolute 64-bit reference time.
    ref_time_64 += (wrapped_count * 0xFFFFFFFFLL);

    /* set linux sync point 50us later than DC sync, just as example */
    delta = (ref_time_64 - 50000) % cycle_time;
    if (delta > (cycle_time / 2))
    {
        delta = delta - cycle_time;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    *offset_time = -(delta / 100) - (integral / 20);

    // Deal with 32 bit overflows
    timeDeltaSystemToDcClock = (delta);

    ec_statecheck(0, EC_STATE_OPERATIONAL, 10);

    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        // Update deltas array
        for (i = 0; i < (100 - 1); i++)
        {
            deltas[i] = deltas[i + 1];
        }
        deltas[99] = delta;

        // Now check whether we had 100 deltas within threshold
        for (i = 0; i < 100; i++)
        {
            if (!((deltas[i] < ETHERCAT_DC_CLOCK_STABLE_THRESHOLD_NS) &&
                  (deltas[i] > -ETHERCAT_DC_CLOCK_STABLE_THRESHOLD_NS)))
            {
                found_dt_value_outside_threshold = 1;
                break;
            }
        }

        // if clock stabilised, request OP mode of all slaves
        if (!found_dt_value_outside_threshold)
        {
            //    refLogger.LogAndPrint(CLogger::Debug, "Request operational state for all EtherCAT slaves");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            // request OP state for all slaves
            ec_writestate(0);
        }
    }

    if (!allSlavesAreInOperationalMode && ((ec_slave[0].state == EC_STATE_OPERATIONAL)))
    {
        bool all_slaves_are_operational = true;
        uint16_t slave_id = 0;

        for (slave_id = 1; slave_id < ec_slavecount; slave_id++)
        {
            if (ec_slave[slave_id].state != EC_STATE_OPERATIONAL)
            {
                // all_slaves_are_operational = false;
                // break;
                ec_slave[slave_id].state = EC_STATE_OPERATIONAL;

                // request OP state for slave
                ec_writestate(slave_id);
            }
        }

        if (all_slaves_are_operational)
        {
            // refLogger.LogAndPrint(CLogger::Info, "Operational state reached for all EtherCAT slaves.");
            allSlavesAreInOperationalMode = true;
        }
        // else
        // {
        // 	refLogger.LogAndPrint(CLogger::Warning, "Slave ID:%u first found not in OPERATIONAL state. State=%u",
        // slave_id, ec_slave[slave_id].state);
        // }
    }

    if (ec_slave[0].hasdc)
    {
        if (timeDeltaSystemToDcClock < min_dt)
        {
            min_dt = timeDeltaSystemToDcClock;
        }

        if (timeDeltaSystemToDcClock > max_dt)
        {
            max_dt = timeDeltaSystemToDcClock;
        }
    }

    // Update historic ref time
    previous_ref_time = reference_time;
}

/**
 * @brief Sends the CoE process data.
 *
 * This function sends the CoE process data to the slaves.
 */
void EthercatMaster::SendCoeProcessData()
{
    int retVal;
    int ec_send_status = 0;

    // CLogger& refLogger = GetLogger();

    retVal = pthread_mutex_lock(&EthercatMaster::process_data_mutex);
    // if (retVal)
    // {
    // 	refLogger.LogAndPrint(CLogger::Error, "EthercatMaster::SendCoeProcessData: mutex lock failed");
    // 	// TODO - this error needs to be returned or passed to caller or server.
    // }

    ec_send_status = ec_send_processdata();

    retVal = pthread_mutex_unlock(&EthercatMaster::process_data_mutex);
    // if (retVal)
    // {
    // 	refLogger.LogAndPrint(CLogger::Error, "EthercatMaster::SendCoeProcessData: mutex unlock failed");
    // 	// TODO - this error needs to be returned or passed to caller or server.
    // }

    // if (ec_send_status <= 0)
    // 	refLogger.LogAndPrint(CLogger::Error, "EthercatMaster::SendCoeProcessData() failed");
}

/**
 * @brief Sends the CoE process data without using a mutex.
 *
 * This function sends the CoE process data without acquiring a mutex lock. It calls the ec_send_processdata() function
 * to send the process data and checks the return value. If the return value is less than or equal to 0, it logs an
 * error message using the GetLogger() function.
 */
void EthercatMaster::SendCoeProcessDataNoMutex()
{
    int ec_send_status = 0;

    // CLogger& refLogger = GetLogger();

    ec_send_status = ec_send_processdata();
    // if (ec_send_status <= 0)
    // 	refLogger.LogAndPrint(CLogger::Error, "EthercatMaster::SendCoeProcessDataNoMutex() failed\n");
}

/**
 * @brief Receives the CoE process data.
 *
 * This function receives the CoE process data from the slaves.
 *
 * @return The number of slaves working.
 */
uint16_t EthercatMaster::ReceiveCoeProcessData()
{
    uint16_t slaves_working_counter = 0;
    int retVal;

    // CLogger& refLogger = GetLogger();

    retVal = pthread_mutex_lock(&EthercatMaster::process_data_mutex);
    if (retVal)
    {
        //  refLogger.LogAndPrint(CLogger::Error, "EthercatMaster::ReceiveCoeProcessData(): mutex lock failed\n");
        return slaves_working_counter;
    }

    slaves_working_counter = ec_receive_processdata(EC_TIMEOUTRET);

    retVal = pthread_mutex_unlock(&EthercatMaster::process_data_mutex);
    if (retVal)
    {
        // refLogger.LogAndPrint(CLogger::Error, "EthercatMaster::ReceiveCoeProcessData(): mutex lock failed\n");
        return slaves_working_counter;
    }

    // if (slaves_working_counter <= 0)
    // 	refLogger.LogAndPrint(CLogger::Error, "EthercatMaster::ReceiveCoeProcessData() failed\n");

    return slaves_working_counter;
}

/**
 * @brief Receives the CoE process data without using a mutex.
 *
 * This function receives the CoE process data without acquiring a mutex lock. It calls the ec_receive_processdata()
 * function to receive the process data and checks the return value. If the return value is less than or equal to 0, it
 * logs an error message using the GetLogger() function.
 *
 * @return The number of slaves working.
 */
void EthercatMaster::RunSlavesWatchdog()
{
    uint8_t slave_id;
    uint8_t currentgroup = 0;
    uint16_t temperature_reading_sleep_counter = 0;

    // CLogger& refLogger = GetLogger();

    while (1)
    {
        // Run the watchdog at ETHERCAT_WATCHDOG_INTERVAL_US
        std::this_thread::sleep_for(std::chrono::microseconds(ETHERCAT_WATCHDOG_INTERVAL_US));

        if (allSlavesAreInOperationalMode &&
            ((currentSlavesWorkingCounter < expectedslavesWorkingCounter) || ec_group[currentgroup].docheckstate))
        {
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = false;

            // Read states of all slaves
            ec_readstate();

            // Check if all slaves are in operational mode
            for (slave_id = 1; slave_id <= ec_slavecount; slave_id++)
            {
                // Check if slave is in the current group
                if ((ec_slave[slave_id].group == currentgroup) && (ec_slave[slave_id].state != EC_STATE_OPERATIONAL))
                {
                    // Slave is not in operational mode
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave_id].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        // refLogger.LogAndPrint(CLogger::Error, "slave_id %d is in SAFE_OP + ERROR, attempting ack.",
                        // slave_id);
                        ec_slave[slave_id].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave_id);
                    }
                    else if (ec_slave[slave_id].state == EC_STATE_SAFE_OP)
                    {
                        // refLogger.LogAndPrint(CLogger::Warning, "slave_id %d is in SAFE_OP, change to OPERATIONAL.",
                        // slave_id);
                        ec_slave[slave_id].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave_id);
                    }
                    else if (ec_slave[slave_id].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave_id, ETHERCAT_SLAVE_RECOVERY_TIMEOUT))
                        {
                            ec_slave[slave_id].islost = FALSE;
                            // refLogger.LogAndPrint(CLogger::Info, "slave_id %d reconfigured",slave_id);
                        }
                    }
                    // Slave is not lost
                    else if (!ec_slave[slave_id].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave_id, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave_id].state == EC_STATE_NONE)
                        {
                            ec_slave[slave_id].islost = TRUE;
                            // refLogger.LogAndPrint(CLogger::Error, "slave_id %d lost",slave_id);
                        }
                    }
                }
                // slave is lost
                if (ec_slave[slave_id].islost)
                {
                    if (ec_slave[slave_id].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave_id, ETHERCAT_SLAVE_RECOVERY_TIMEOUT))
                        {
                            ec_slave[slave_id].islost = FALSE;
                            // refLogger.LogAndPrint(CLogger::Info, "slave_id %d recovered",slave_id);
                        }
                    }
                    else
                    {
                        ec_slave[slave_id].islost = FALSE;
                        // refLogger.LogAndPrint(CLogger::Info, "slave_id %d found",slave_id);
                    }
                }
            }

            // if(!ec_group[currentgroup].docheckstate)
            // 	refLogger.LogAndPrint(CLogger::Info, "OK: all slaves resumed OPERATIONAL.");
        }
        // If not all slaves are in operational mode, print the states of all slaves
        else if (!allSlavesAreInOperationalMode)
        {
            uint16_t i = 0;

            // PrintMessage(MSG_DEBUG, "Not all EtherCAT slaves have reached operational state.\n");

            // Read states of all slaves
            ec_readstate();

            // for(i = 1; i <= ec_slavecount; i++)
            // {
            // 	if(ec_slave[i].state != EC_STATE_OPERATIONAL)
            // 	{
            // 		refLogger.LogAndPrint(CLogger::Debug, "EtherCAT Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s",
            // 				i,
            // 				ec_slave[i].state,
            // 				ec_slave[i].ALstatuscode,
            // 				ec_ALstatuscode2string(ec_slave[i].ALstatuscode)
            // 				);
            // 	}
            // }
        }
    }
}

/** This function will return a 16 bit field, where a 1 represents the position of a slave of the passed in type in
 * ec_slave */
uint16_t EthercatMaster::getEcSlaveIndicesOfType(ETHERCAT_SLAVE_TYPE type)
{
    uint16_t bitfield = 0;
    uint8_t i = 0;

    for (i = 0; i < MAX_ETHERCAT_SLAVE_COUNT; i++)
    {
        if (slaveTypes[i] == type)
        {
            bitfield |= (1u << i);
        }
    }

    return bitfield;
}
