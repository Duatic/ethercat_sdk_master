#pragma once

#include <ethercat_sdk_master/EthercatMaster.hpp>
#include <map>

namespace ecat_master {
    /**
     *  @brief Provides the only method how we can use the same ethercat bus in multiple ros2control hardware interfaces
     * The idea is that we centrally manage the instances of the EthercatMasters and each hardware interface may attach its devices to it
    */
    class EthercatMasterSingleton {
        public:
            static EthercatMasterSingleton&  instance(); //Method has to be in cpp file in order to work properly

            /**
             * @brief get a shared pointer to an ecat master instance
             * If there is already a master active for the given instance we simply reuse it and return it
             * @note Using this methods enforces asynchronous spinning of the master in this class
             * @note In case the "new" ethercat master configuration does not match the existing one only a warning is printed!
             */
            std::shared_ptr<EthercatMaster> get(const EthercatMasterConfiguration& config, int rt_prio = 48) {
                std::lock_guard<std::recursive_mutex> guard(lock_);

                if(ecat_masters_.find(config.networkInterface) == ecat_masters_.end())
                {
                    MELO_INFO_STREAM("Setting up new EthercatMaster on interface: " << config.networkInterface << " and updating it");
                    auto master = std::make_shared<EthercatMaster>();
                    master->loadEthercatMasterConfiguration(config);

                    ecat_masters_[config.networkInterface] = master;
                    
                    //Spin the master asynchronously
                    spin_threads_.emplace(config.networkInterface, std::make_unique<std::thread>(std::bind(&EthercatMasterSingleton::spin,this, std::placeholders::_1, std::placeholders::_2), master, rt_prio));
                }
                
                if(config != ecat_masters_[config.networkInterface]->getConfiguration()){
                    //Print warning or abort if the configuration does not match!
                    MELO_WARN_STREAM("Ethercat master configurations do not match for bus: " << config.networkInterface);
                }

                return ecat_masters_.at(config.networkInterface);
            }

            bool hasMaster(const EthercatMasterConfiguration& config){
                return ecat_masters_.find(config.networkInterface) != ecat_masters_.end();
            }
            bool hasMaster(const std::string& networkInterface) {
                return ecat_masters_.find(networkInterface) != ecat_masters_.end();
            }

            std::shared_ptr<EthercatMaster> operator[] (const EthercatMasterConfiguration& config){
                return get(config);
            }

            bool shutdown(std::shared_ptr<EthercatMaster> master){
                return true;
            }
        private:

        EthercatMasterSingleton(){

        }
        ~EthercatMasterSingleton() {
            abort_ = true;

            for(const auto& [interface, thread]: spin_threads_){
                thread->join();
            }
            for(const auto &  [interfrace, master]: ecat_masters_){
                master->preShutdown();
                master->shutdown();
            }
        }
        void spin(std::shared_ptr<EthercatMaster> master_, int rt_prio){
            //We override the default rt prio of 99 as this might starve kernel threads. 
            master_->setRealtimePriority(rt_prio); 
            while(!abort_){
                master_->update(UpdateMode::StandaloneEnforceRate);
            }
        }

        std::map<std::string,std::shared_ptr<EthercatMaster>> ecat_masters_;
        std::map<std::string, std::unique_ptr<std::thread>> spin_threads_;
        std::map<std::string, std::atomic_bool> abort_signals_;
        std::recursive_mutex lock_;
        std::atomic_bool abort_ = false;
    };

}