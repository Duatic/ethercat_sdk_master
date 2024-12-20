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
    public:
        static EthercatMasterSingleton &instance(); // Method has to be in cpp file in order to work properly

        /**
         * @brief get a shared pointer to an ecat master instance
         * If there is already a master active for the given instance we simply reuse it and return it
         * @note Using this methods enforces asynchronous spinning of the master in this class
         * @note In case the "new" ethercat master configuration does not match the existing one only a warning is printed!
         */
        std::shared_ptr<EthercatMaster> aquireMaster(const EthercatMasterConfiguration &config, int rt_prio = 48)
        {
            // Give it at least some thread safety
            std::lock_guard<std::mutex> guard(lock_);

            // Check if we already have a master up and running for the given networkinterface
            if (ecat_masters_.find(config.networkInterface) == ecat_masters_.end())
            {
                MELO_INFO_STREAM("Setting up new EthercatMaster on interface: " << config.networkInterface << " and updating it");
                auto master = std::make_shared<EthercatMaster>();
                master->loadEthercatMasterConfiguration(config);

                ecat_masters_[config.networkInterface] = master;
                abort_signals_[config.networkInterface] = false;
                reference_count_[config.networkInterface] = 1;
                // Spin the master asynchronously
                spin_threads_.emplace(config.networkInterface, std::make_unique<std::thread>(std::bind(&EthercatMasterSingleton::spin, this, std::placeholders::_1, std::placeholders::_2), master, rt_prio));
            }
            else
            {
                // In case we already handle the network interface we just increate its reference counter and return the instance
                reference_count_[config.networkInterface] += 1;
            }

            if (config != ecat_masters_[config.networkInterface]->getConfiguration())
            {
                // Print warning or abort if the configuration does not match!
                MELO_WARN_STREAM("Ethercat master configurations do not match for bus: " << config.networkInterface);
            }

            return ecat_masters_.at(config.networkInterface);
        }

        bool hasMaster(const EthercatMasterConfiguration &config)
        {
            return ecat_masters_.find(config.networkInterface) != ecat_masters_.end();
        }
        bool hasMaster(const std::string &networkInterface)
        {
            return ecat_masters_.find(networkInterface) != ecat_masters_.end();
        }

        /**
         * @brief releaseMaster - release your handle obtain via aquireMaster
         * This method decrements the internal reference counter for the given EthercatMaster and performs the shutdown if no references are living anymore
         */
        bool releaseMaster(const std::shared_ptr<EthercatMaster> &master)
        {
            // Give it at least some thread safety
            std::lock_guard<std::mutex> guard(lock_);

            const auto &network_interface = master->getConfiguration().networkInterface;
            // First check if we even handle this ethercat master
            if (!hasMaster(network_interface))
            {
                throw std::logic_error("EthercatMaster for interface: " + network_interface + " is not handled by this singleton");
            }

            // Decrement the reference counter and check if it is zero
            reference_count_[network_interface] -= 1;

            if (reference_count_[network_interface] <= 0)
            {
                // Perform the actual shutdown if all callers have called shutdown on their reference
                shutdownMaster(master);
                return true;
            }

            return false;
        }
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

            // Tell the update thread of the corresponding master to stop spinning
            abort_signals_[network_interface] = true;

            // Wait for the thread to end
            spin_threads_[network_interface]->join();

            // Perform the actuall shutdown
            ecat_masters_[network_interface]->preShutdown(set_to_safe_op);
            ecat_masters_[network_interface]->shutdown();

            // And remove all entries
            abort_signals_.erase(network_interface);
            spin_threads_.erase(network_interface);
            ecat_masters_.erase(network_interface);
            reference_count_.erase(network_interface);
        }

        EthercatMasterSingleton() = default;
        ~EthercatMasterSingleton()
        {
            // Tell every update thread to stop spinning
            for (auto &[interface, abort_flag] : abort_signals_)
            {
                abort_flag = true;
            }
            // Wait for the threads to end
            for (const auto &[interface, thread] : spin_threads_)
            {
                thread->join();
            }
            for (const auto &[interfrace, master] : ecat_masters_)
            {
                master->preShutdown();
                master->shutdown();
            }
        }
        void spin(const std::shared_ptr<EthercatMaster>& master_, int rt_prio)
        {

            // Obtain a reference to the abort floag
            auto &abort_flag = abort_signals_[master_->getConfiguration().networkInterface];
            // We override the default rt prio of 99 as this might starve kernel threads.
            master_->setRealtimePriority(rt_prio);
            while (!abort_flag)
            {
                master_->update(UpdateMode::StandaloneEnforceRate);
            }
        }

        std::map<std::string, std::shared_ptr<EthercatMaster>> ecat_masters_;
        std::map<std::string, std::unique_ptr<std::thread>> spin_threads_;
        std::map<std::string, std::atomic_bool> abort_signals_;
        std::map<std::string, int> reference_count_;

        std::mutex lock_;
    };

}