/**
 * perls2_redis_client.h
 *
 * Copyright 2021. All Rights Reserved.
 *
 * Created: January 21, 2021
 * Authors: Toki Migimatsu
 *          Rohun Kulkarni
 * 
 * extends ctrl_utils/redis_client.h functionality for perls2.
 */

#ifndef PERLS2_REDIS_CLIENT_H_
#define PERLS2_REDIS_CLIENT_H_

#include <iostream>
#include <fstream> // std::fstream

#include <ctrl_utils/redis_client.h>
#include <cpp_redis/cpp_redis>

namespace ctrl_utils {
    class Perls2RedisClient : public RedisClient {

        public: 
            Perls2RedisClient() : RedisClient() {}

            void auth(const std::string& passfile){
                std::ifstream pf(passfile);
                std::string pass;
                if (pf) {
                  getline(pf, pass);
                }
                cpp_redis::client::auth(pass, [](const cpp_redis::reply& reply) {
                  if (reply.is_error()) { std::cerr << "Authentication failed: " << reply.as_string() << std::endl; }
                  else {
                    std::cout << "successful authentication" << std::endl;
                  }
                });
            }
        };

}   // namespace ctrl_utils

#endif // PERLS2_REDIS_CLIENT_H_