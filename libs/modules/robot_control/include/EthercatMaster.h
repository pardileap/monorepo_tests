#pragma once
#include <pthread.h>
#include <soem/ethercat.h>
#include <stdint.h>

/**< Be very careful to change this as this is directly related to a 16-bit bitfield */
#define MAX_ETHERCAT_SLAVE_COUNT (sizeof(uint16_t) * 8)

class EthercatMaster
{
public:
    static constexpr unsigned int ETHERCAT_SLAVES_IO_MAPS_BUFFER_SIZE = 4096U;
    static constexpr int64_t ETHERCAT_DC_CLOCK_STABLE_THRESHOLD_NS = 100000LL;
    static constexpr unsigned int ETHERCAT_BUS_UPDATE_INTERVAL_US = 4000U;
    static constexpr unsigned int ETHERCAT_SLAVE_RECOVERY_TIMEOUT = 500U;
    static constexpr unsigned int ETHERCAT_WATCHDOG_INTERVAL_US = 100000U;

    // TODO(): move out of this class
    enum ETHERCAT_SLAVE_TYPE
    {
        ETHERCAT_SLAVE_INVALID = 0,
        ETHERCAT_SLAVE_TYPE_ZEROERR
    };

    static pthread_mutex_t process_data_mutex;

    EthercatMaster();
    ~EthercatMaster() { TerminateComms(); };

    static bool InitialiseComms(const char* const nic_name);
    static void TerminateComms();
    static void SendCoeProcessData();
    static void SendCoeProcessDataNoMutex();
    static uint16_t ReceiveCoeProcessData();
    static void SetSlavesWorkingCounter(uint8_t working_counter)
    {
        EthercatMaster::currentSlavesWorkingCounter = working_counter;
    }
    static void SyncSystemTimeToBusDcTime(int64_t reference_time, int64_t cycle_time, int64_t* offset_time);
    static bool BusHasDistributedClock() { return ec_slave[0].hasdc; }
    static bool IsSlaveOperational(uint8_t slave_id) { return (ec_slave[slave_id].state == EC_STATE_OPERATIONAL); }
    static bool AreAllSlavesOperational() { return (ec_slave[0].state == EC_STATE_OPERATIONAL); }
    // TODO(): this function has ZeroErr specific needs rework or to be moved to zeroerr specific class
    static uint8_t FindSlavesNumber();
    static uint8_t ConfigureSlave(
        uint8_t slave_id, uint32 ecat_manufacturer_id, uint32 ecat_id, int (*reg_config_func)(uint16),
        ETHERCAT_SLAVE_TYPE slave_type
    );
    static uint8_t ConfigureSlavesDC(uint32_t ecat_bus_interval_ms);
    // Populate expected slaves work counter based on discovered slaves
    static void PopulateSlavesWorkingCounter()
    {
        expectedslavesWorkingCounter = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    };
    static bool ConfigureSlavesIOMap();
    static void RunSlavesWatchdog();

    static bool SlavesAreInOperationalMode() { return allSlavesAreInOperationalMode; }
    static bool IsSlaveReady(uint8_t slave_id);

    static bool IsEthercatInitialisedAndMapped() { return m_bEcatInitialisedAndMapped; }
    static void SetEthercatInitialisedAndMapped(bool is_initialised_and_mapped)
    {
        m_bEcatInitialisedAndMapped = is_initialised_and_mapped;
    }

    static uint16_t getEcSlaveIndicesOfType(ETHERCAT_SLAVE_TYPE type);

private:
    static uint8_t IOmap[ETHERCAT_SLAVES_IO_MAPS_BUFFER_SIZE]; /**< Buffer that holds all slaves' IO maps */
    static uint8_t currentSlavesWorkingCounter;                /**< Used by eCat Master to monitor slaves'
                                                                topology / changes in topology */
    static uint8_t expectedslavesWorkingCounter;               /**< Used by eCat Master to monitor slaves'
                                                                topology / changes in topology */
    static int64_t timeDeltaSystemToDcClock;                   /**< Difference between bus DC clock and System clock */
    static bool allSlavesAreInOperationalMode; /**< A flag indicating that all slaves are in operational mode */
    static bool m_bEcatInitialisedAndMapped;   /**< A flag indicating Ethercat has been initialised and process data has
                                                been mapped */
    static ETHERCAT_SLAVE_TYPE slaveTypes[MAX_ETHERCAT_SLAVE_COUNT]; /**< An array containing the slave types as
                                                                        discovered and manually configured in SOEM */
};
