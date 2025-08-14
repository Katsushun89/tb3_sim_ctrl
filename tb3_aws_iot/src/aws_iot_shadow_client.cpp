#include "aws_iot_shadow_client.hpp"
#include <fstream>
#include <sstream>

AwsIotShadowClient::AwsIotShadowClient(const std::string& endpoint, 
                                       const std::string& thing_name,
                                       const std::string& cert_path,
                                       const std::string& key_path,
                                       const std::string& ca_path)
: endpoint_(endpoint), thing_name_(thing_name), connected_(false)
{
  // Initialize OpenSSL
  SSL_library_init();
  SSL_load_error_strings();
  OpenSSL_add_all_algorithms();

  // Create SSL context
  ssl_ctx_ = SSL_CTX_new(TLS_client_method());
  if (!ssl_ctx_) {
    throw std::runtime_error("Failed to create SSL context");
  }

  // Load certificates
  if (SSL_CTX_load_verify_locations(ssl_ctx_, ca_path.c_str(), nullptr) != 1) {
    throw std::runtime_error("Failed to load CA certificate");
  }

  if (SSL_CTX_use_certificate_file(ssl_ctx_, cert_path.c_str(), SSL_FILETYPE_PEM) != 1) {
    throw std::runtime_error("Failed to load device certificate");
  }

  if (SSL_CTX_use_PrivateKey_file(ssl_ctx_, key_path.c_str(), SSL_FILETYPE_PEM) != 1) {
    throw std::runtime_error("Failed to load private key");
  }

  // Verify private key
  if (SSL_CTX_check_private_key(ssl_ctx_) != 1) {
    throw std::runtime_error("Private key does not match certificate");
  }

  SSL_CTX_set_verify(ssl_ctx_, SSL_VERIFY_PEER, nullptr);
}

AwsIotShadowClient::~AwsIotShadowClient()
{
  disconnect();
  if (ssl_ctx_) {
    SSL_CTX_free(ssl_ctx_);
  }
  EVP_cleanup();
}

bool AwsIotShadowClient::connect()
{
  // Create socket
  struct addrinfo hints{}, *result;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  if (getaddrinfo(endpoint_.c_str(), "8883", &hints, &result) != 0) {
    return false;
  }

  sockfd_ = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
  if (sockfd_ < 0) {
    freeaddrinfo(result);
    return false;
  }

  if (::connect(sockfd_, result->ai_addr, result->ai_addrlen) != 0) {
    close(sockfd_);
    freeaddrinfo(result);
    return false;
  }

  freeaddrinfo(result);

  // Create SSL connection
  ssl_ = SSL_new(ssl_ctx_);
  if (!ssl_) {
    close(sockfd_);
    return false;
  }

  SSL_set_fd(ssl_, sockfd_);

  // Set SNI
  SSL_set_tlsext_host_name(ssl_, endpoint_.c_str());

  if (SSL_connect(ssl_) != 1) {
    SSL_free(ssl_);
    close(sockfd_);
    return false;
  }

  // Send MQTT CONNECT packet
  if (!send_mqtt_connect()) {
    disconnect();
    return false;
  }

  // Wait for CONNACK
  if (!receive_connack()) {
    disconnect();
    return false;
  }

  connected_ = true;
  return true;
}

void AwsIotShadowClient::disconnect()
{
  if (ssl_) {
    SSL_shutdown(ssl_);
    SSL_free(ssl_);
    ssl_ = nullptr;
  }
  
  if (sockfd_ >= 0) {
    close(sockfd_);
    sockfd_ = -1;
  }
  
  connected_ = false;
}

bool AwsIotShadowClient::update_shadow(const std::string& shadow_json)
{
  if (!connected_) {
    return false;
  }

  std::string topic = "$aws/things/" + thing_name_ + "/shadow/update";
  return publish_message(topic, shadow_json);
}

bool AwsIotShadowClient::send_mqtt_connect()
{
  // Simplified MQTT CONNECT packet
  std::string client_id = thing_name_;
  
  std::vector<uint8_t> packet;
  
  // Fixed header
  packet.push_back(0x10); // CONNECT packet type
  
  // Variable header
  std::vector<uint8_t> variable_header;
  
  // Protocol Name
  std::string protocol_name = "MQTT";
  variable_header.push_back(0x00);
  variable_header.push_back(0x04);
  for (char c : protocol_name) {
    variable_header.push_back(c);
  }
  
  // Protocol Level
  variable_header.push_back(0x04); // MQTT 3.1.1
  
  // Connect Flags
  variable_header.push_back(0x02); // Clean session
  
  // Keep Alive
  variable_header.push_back(0x00);
  variable_header.push_back(0x3C); // 60 seconds
  
  // Payload
  std::vector<uint8_t> payload;
  
  // Client ID
  payload.push_back((client_id.length() >> 8) & 0xFF);
  payload.push_back(client_id.length() & 0xFF);
  for (char c : client_id) {
    payload.push_back(c);
  }
  
  // Remaining length
  uint32_t remaining_length = variable_header.size() + payload.size();
  encode_remaining_length(packet, remaining_length);
  
  // Append variable header and payload
  packet.insert(packet.end(), variable_header.begin(), variable_header.end());
  packet.insert(packet.end(), payload.begin(), payload.end());
  
  return SSL_write(ssl_, packet.data(), packet.size()) > 0;
}

bool AwsIotShadowClient::receive_connack()
{
  uint8_t buffer[4];
  int received = SSL_read(ssl_, buffer, sizeof(buffer));
  
  if (received >= 4 && buffer[0] == 0x20 && buffer[1] == 0x02) {
    return buffer[3] == 0x00; // Connection accepted
  }
  
  return false;
}

bool AwsIotShadowClient::publish_message(const std::string& topic, const std::string& message)
{
  std::vector<uint8_t> packet;
  
  // Fixed header
  packet.push_back(0x30); // PUBLISH packet type
  
  // Variable header
  std::vector<uint8_t> variable_header;
  
  // Topic name
  variable_header.push_back((topic.length() >> 8) & 0xFF);
  variable_header.push_back(topic.length() & 0xFF);
  for (char c : topic) {
    variable_header.push_back(c);
  }
  
  // Payload (message)
  std::vector<uint8_t> payload(message.begin(), message.end());
  
  // Remaining length
  uint32_t remaining_length = variable_header.size() + payload.size();
  encode_remaining_length(packet, remaining_length);
  
  // Append variable header and payload
  packet.insert(packet.end(), variable_header.begin(), variable_header.end());
  packet.insert(packet.end(), payload.begin(), payload.end());
  
  return SSL_write(ssl_, packet.data(), packet.size()) > 0;
}

void AwsIotShadowClient::encode_remaining_length(std::vector<uint8_t>& packet, uint32_t length)
{
  do {
    uint8_t encoded_byte = length % 128;
    length = length / 128;
    if (length > 0) {
      encoded_byte = encoded_byte | 128;
    }
    packet.push_back(encoded_byte);
  } while (length > 0);
}