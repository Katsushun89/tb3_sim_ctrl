#pragma once

#include <string>
#include <vector>
#include <memory>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

class AwsIotShadowClient
{
public:
  AwsIotShadowClient(const std::string& endpoint, 
                     const std::string& thing_name,
                     const std::string& cert_path,
                     const std::string& key_path,
                     const std::string& ca_path);
  
  ~AwsIotShadowClient();
  
  bool connect();
  void disconnect();
  bool is_connected() const { return connected_; }
  
  bool update_shadow(const std::string& shadow_json);

private:
  std::string endpoint_;
  std::string thing_name_;
  
  SSL_CTX* ssl_ctx_;
  SSL* ssl_;
  int sockfd_;
  bool connected_;
  
  bool send_mqtt_connect();
  bool receive_connack();
  bool publish_message(const std::string& topic, const std::string& message);
  void encode_remaining_length(std::vector<uint8_t>& packet, uint32_t length);
};