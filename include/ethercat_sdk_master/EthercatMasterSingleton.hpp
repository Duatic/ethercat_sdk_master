#pragma once

#include <ethercat_sdk_master/EthercatMaster.hpp>
#include <map>

namespace ecat_master
{
    /**
     *  @brief Provides the only method how we can use the same ethercat bus in multiple ros2control hardware interfaces
     * The idea is that we centrally manage the instances of the EthercatMasters and each hardware interface may attach its devices to it
     */
    class EthercatMasterSingleton
    {
    private:
        // This represents and internal handle which contains all necessary information in order to manage multiple EthercatMasterInstances
        struct InternalHandle
        {
            std::shared_ptr<EthercatMaster> ecat_master;
            std::unique_ptr<std::thread> spin_thread;
            std::atomic_bool abort_signal{false};
            int reference_count{0};
            std::map<int, bool> handles_ready;
            int rt_prio{48};
            InternalHandle(const std::shared_ptr<EthercatMaster> &ecat_master_, std::unique_ptr<std::thread> spin_thread_, int rt_prio_) : ecat_master(ecat_master_), spin_thread(std::move(spin_thread_)), rt_prio(rt_prio_)
            {
            }
            InternalHandle(InternalHandle &&o) : ecat_master(o.ecat_master), spin_thread(std::move(o.spin_thread)), abort_signal(o.abort_signal.load()), reference_count(o.reference_count), handles_ready(o.handles_ready), rt_prio(o.rt_prio) {}
        };

    public:
        // This represents a public handle which can be used to uniquely identify the result of an aquireMaster operation
        struct Handle
        {
            int id{0};
            std::shared_ptr<EthercatMaster> ecat_master;
        };

        static EthercatMasterSingleton &instance(); // Method has to be in cpp file in order to work properly

        /**
         * @brief get a shared pointer to an ecat master instance
         * If there is already a master active for the given instance we simply reuse it and return it
         * @note Using this methods enforces asynchronous spinning of the master in this class
         * @note In case the "new" ethercat master configuration does not match the existing one only a warning is printed!
         */
        Handle aquireMaster(const EthercatMasterConfiguration &config, int rt_prio = 48)
        {
            // Give it at least some thread safety
            std::lock_guard<std::mutex> guard(lock_);

            // Check if we already have a master up and running for the given networkinterface
            if (handles_.find(config.networkInterface) == handles_.end())
            {
                MELO_INFO_STREAM("Setting up new EthercatMaster on interface: " << config.networkInterface << " and updating it");
                auto master = std::make_shared<EthercatMaster>();
                master->loadEthercatMasterConfiguration(config);

                handles_.emplace(config.networkInterface, InternalHandle{
                                                              master, nullptr, rt_prio});
            }

            // Increment its reference counter
            handles_.at(config.networkInterface).reference_count += 1;
            // Mark the new handle as not ready
            handles_.at(config.networkInterface).handles_ready.emplace(handles_.at(config.networkInterface).reference_count, false);
            if (config != handles_.at(config.networkInterface).ecat_master->getConfiguration())
            {
                // Print warning or abort if the configuration does not match!
                MELO_WARN_STREAM("Ethercat master configurations do not match for bus: " << config.networkInterface);
            }

            // Create and return a handle in which we use the reference count as an id
            return Handle{handles_.at(config.networkInterface).reference_count, handles_.at(config.networkInterface).ecat_master};
        }
        /**
         * @brief mark a specific handle as ready. If all handles aquired via aquireMaster are ready the bus gets activated and is spun in a separate thread
         * @return true if the ethercat is now activated
         */
        bool markAsReady(const Handle &handle)
        {
            // 1. find the corresponding internal handle
            const auto &network_interface = handle.ecat_master->getConfiguration().networkInterface;

            if (!hasMaster(network_interface))
            {
                throw std::logic_error("EthercatMaster for interface: " + network_interface + " is not handled by this singleton");
            }

            auto &internal_handle = handles_.at(network_interface);

            // 2. Check if the handle is already marked as ready

            if (internal_handle.handles_ready.at(handle.id))
            {
                throw std::runtime_error("Handle with id: " + std::to_string(handle.id) + " on interface: " + network_interface + " was already marked as ready!");
            }

            // 3. Mark it as ready
            internal_handle.handles_ready.at(handle.id) = true;

            // 4. Check if all handles are ready

            bool all_ready = true;
            for (auto &[id, ready] : internal_handle.handles_ready)
            {
                if (!ready)
                {
                    all_ready = false;
                    break;
                }
            }

            if (!all_ready)
            {   
                MELO_INFO_STREAM("Not all handles ready - defering start");
                return false;
            }

            // 5. Perform the startup and spin
            if (!internal_handle.ecat_master->startup())
            {
                throw std::runtime_error("Could not startup ethercat master on interface: " + network_interface);
            }
            MELO_INFO_STREAM("Starting asynchronous worker thread for ethercat master on network interface: "  << network_interface);
            // Spin the master asynchronously
            internal_handle.spin_thread = std::make_unique<std::thread>(std::bind(&EthercatMasterSingleton::spin, this, std::placeholders::_1),network_interface);
            return true;
        }
        /**
         * @brief check if an ethercat master is active and managed by this implementation for the given configuration
         * @note only checks the interface name
         */
        bool hasMaster(const EthercatMasterConfiguration &config)
        {
            return handles_.find(config.networkInterface) != handles_.end();
        }
        bool hasMaster(const std::string &networkInterface)
        {
            return handles_.find(networkInterface) != handles_.end();
        }

        /**
         * @brief releaseMaster - release your handle obtain via aquireMaster
         * This method decrements the internal reference counter for the given EthercatMaster and performs the shutdown if no references are living anymore
         */
        bool releaseMaster(const Handle &handle)
        {
            // Give it at least some thread safety
            std::lock_guard<std::mutex> guard(lock_);
            
          
            const auto &network_interface = handle.ecat_master->getConfiguration().networkInterface;
            // First check if we even handle this ethercat master
            if (!hasMaster(network_interface))
            {
                throw std::logic_error("EthercatMaster for interface: " + network_interface + " is not handled by this singleton");
            }

            // Decrement the reference counter and check if it is zero
            handles_.at(network_interface).reference_count -= 1;

            if (handles_.at(network_interface).reference_count <= 0)
            {
                MELO_INFO_STREAM("Shutting down EthercatMaster for interface: " << network_interface );
                // Perform the actual shutdown if all callers have called shutdown on their reference
                shutdownMaster(handle.ecat_master);
                return true;
            }

            return false;
        }
        /**
         * @brief shutdown a given EthercatMaster even though the reference counter is not 0
         * @note this is really unsafe as other instances using the ethercat master will most likely crash
         * But ins some cases it is better to properly close the bus than not to crash the control program on the PC
         */
        void forceShutdownMaster(const std::shared_ptr<EthercatMaster> &master)
        {

            // Give it at least some thread safety
            std::lock_guard<std::mutex> guard(lock_);

            shutdownMaster(master);
        }

    private:
        void shutdownMaster(const std::shared_ptr<EthercatMaster> &master, bool set_to_safe_op = true)
        {
            const auto &network_interface = master->getConfiguration().networkInterface;
            if (!hasMaster(network_interface))
            {
                throw std::logic_error("EthercatMaster for interface: " + network_interface + " is not handled by this singleton");
            }
            MELO_INFO_STREAM("Shutting down ethercat master: " << network_interface);

            // Tell the update thread of the corresponding master to stop spinning
            handles_.at(network_interface).abort_signal = true;

            // Wait for the thread to end
            handles_.at(network_interface).spin_thread->join();

            // Perform the actual shutdown
            handles_.at(network_interface).ecat_master->preShutdown(set_to_safe_op);
            handles_.at(network_interface).ecat_master->shutdown();

            // And remove all entries
            handles_.erase(network_interface);
        }

        EthercatMasterSingleton() = default;
        ~EthercatMasterSingleton()
        {
            // Tell every update thread to stop spinning
            for (auto &[interface, handle] : handles_)
            {
                handle.abort_signal = true;
            }
            // Wait for the threads to end
            for (const auto &[interface, handle] : handles_)
            {
                handle.spin_thread->join();
            }
            for (const auto &[interfrace, handle] : handles_)
            {
                handle.ecat_master->preShutdown();
                handle.ecat_master->shutdown();
            }
        }
        void spin(const std::string &network_interface)
        {
            auto &handle = handles_.at(network_interface);
            // Obtain a reference to the abort floag
            auto &abort_flag = handle.abort_signal;

            auto &master = handle.ecat_master;
            // We override the default rt prio of 99 as this might starve kernel threads.
            master->setRealtimePriority(handle.rt_prio);

            if (master->activate()) {
                MELO_INFO_STREAM("Activated the Bus: " << master->getBusPtr()->getName());
            }
            while (!abort_flag)
            {
                master->update(UpdateMode::StandaloneEnforceRate);
            }
            master->deactivate();
        }

        std::map<std::string, InternalHandle> handles_;
        std::mutex lock_;
    };

}