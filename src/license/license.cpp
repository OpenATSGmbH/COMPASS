/*
 * This file is part of OpenATS COMPASS.
 *
 * COMPASS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * COMPASS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#include "license.h"

#include "global.h"
#include "logger.h"
#include "files.h"
#include "timeconv.h"

#include "traced_assert.h"

#include <QString>
#include <QByteArray>
#include <QColor>

#include <openssl/pem.h>
#include <openssl/evp.h>
#include <openssl/sha.h>
#include <openssl/err.h>

#if USE_EXPERIMENTAL_SOURCE == true
#include "extra_resources.h"
#endif

namespace license
{

const std::string License::ColorValid   = QColor(34,139,34).name().toStdString();
const std::string License::ColorInvalid = QColor(220,20,60).name().toStdString();
const std::string License::ColorWarning = QColor(255,140,0).name().toStdString();

const boost::posix_time::time_duration D0 = boost::posix_time::hours(24);
const boost::posix_time::time_duration D1 = boost::posix_time::hours(48);

/**
*/
std::string License::typeToString(Type t)
{
    if (t == Type::Free)
        return "Free";
    else if (t == Type::Trial)
        return "Trial";
    else if (t == Type::Pro)
        return "Pro";
    
    return "";
}

/**
*/
boost::optional<License::Type> License::typeFromString(const std::string& s)
{
    if (s == "Free")
        return Type::Free;
    else if (s == "Trial")
        return Type::Trial;
    else if (s == "Pro")
        return Type::Pro;
    
    return {};
}

/**
*/
std::string License::componentToString(Component c)
{
    if (c == ComponentProbIMMReconstructor)
        return "ProbIMMReconstructor";
    
    return "";
}

/**
*/
boost::optional<License::Component> License::componentFromString(const std::string& s)
{
    if (s == "ProbIMMReconstructor")
        return ComponentProbIMMReconstructor;

    return {};
}

/**
*/
boost::optional<nlohmann::json> License::unpackageLicenseKey(const std::string& license_key)
{
    auto bytes = QByteArray::fromHex(QString::fromStdString(license_key).toUtf8());
    std::vector<std::uint8_t> msgpack(bytes.size());
    memcpy(msgpack.data(), bytes.data(), (size_t)bytes.size() * sizeof(std::uint8_t));

    nlohmann::json j;
    
    try
    {
        j = nlohmann::json::from_msgpack(msgpack);
    }
    catch(...)
    {
        return {};
    }
    
    if (!j.is_object())
        return {};

    return j;
}

/**
*/
std::vector<unsigned char> License::licenseBytes(const nlohmann::json& license_json)
{
    if (!license_json.contains("license"))
        return {};

    const nlohmann::json& j = license_json.at("license");

    //license to msgpack
    auto byte_arr = nlohmann::json::to_msgpack(j);

    std::vector<unsigned char> bytes;
    bytes.resize(byte_arr.size());
    memcpy(bytes.data(), byte_arr.data(), byte_arr.size() * sizeof(unsigned char));

    return bytes;
}

/**
*/
std::vector<unsigned char> License::signatureBytes(const std::string& signature)
{
    //hey string to bytes
    auto byte_arr = QByteArray::fromHex(QByteArray::fromStdString(signature));

    std::vector<unsigned char> bytes;
    bytes.resize(byte_arr.count());
    memcpy(bytes.data(), byte_arr.data(), (size_t)byte_arr.count() * sizeof(unsigned char));

    return bytes;
}

/**
*/
std::string License::stringFromValidity(Validity validity)
{
    if (validity == Validity::Empty)
        return "Empty";
    else if (validity == Validity::ReadError)
        return "Unreadable";
    else if (validity == Validity::Invalid)
        return "Invalid";
    else if (validity == Validity::NotActivated)
        return "Not Activated";
    else if (validity == Validity::Expired)
        return "Expired";
    else if (validity == Validity::Valid)
        return "Valid";

    return "";
}

/**
*/
std::string License::colorFromValidity(Validity validity)
{
    if (validity == Validity::Empty)
        return ColorInvalid;
    else if (validity == Validity::ReadError)
        return ColorInvalid;
    else if (validity == Validity::Invalid)
        return ColorInvalid;
    else if (validity == Validity::NotActivated)
        return ColorWarning;
    else if (validity == Validity::Expired)
        return ColorInvalid;
    else if (validity == Validity::Valid)
        return ColorValid;

    return "";
}

/**
*/
bool License::isComplete() const
{
    return (!id.empty() &&
            version > 0 &&
            !signature.empty() &&
            json_blob.is_object());
}

/**
*/
bool License::read(const nlohmann::json& license_json, 
                   const std::string* expected_id)
{
    *this = {};

    auto logReadError = [ & ] (const std::string& err)
    {
        state = State::ReadError;
        error = err;
        return false;
    };

    try
    {
        json_blob = license_json;

        if (!license_json.contains("license") ||
            !license_json.contains("signature") || 
            !license_json.contains("id"))
            return logReadError("license structure invalid");

        signature = license_json.at("signature");
        if (signature.empty())
            return logReadError("license signature missing");

        id = license_json.at("id");

        if (expected_id && id != *expected_id)
            return logReadError("license obtains invalid id");

        const auto& j = license_json.at("license");
        if (!j.is_object())
            return logReadError("license structure invalid");

        if (!j.contains("version"))
            return logReadError("license version missing");

        version = j.at("version");
        if (version < 1)
            return logReadError("license version invalid");

        if (version >= 1)
        {
            if (!j.contains("type") ||
                !j.contains("components") ||
                !j.contains("licensee") ||
                !j.contains("date_activation") ||
                !j.contains("date_expiration") ||
                !j.contains("created") ||
                !j.contains("info"))
                return logReadError("license structure invalid");

            //read type
            {
                if (!typeFromString(j.at("type")).has_value())
                    return logReadError("license type invalid");
                
                type = typeFromString(j.at("type")).value();
            }

            //read components
            {
                const auto& components_json = j.at("components");
                if (!components_json.is_array())
                    return logReadError("component list invalid");

                for(const auto& c : components_json)
                {
                    auto ctype = componentFromString(c);
                    if (!ctype.has_value())
                        return logReadError("component list invalid");

                    components.insert(ctype.value());
                }
            }

            licensee = j.at("licensee");

            date_activation = Utils::Time::fromDateString(j["date_activation"]);
            date_expiration = Utils::Time::fromDateString(j["date_expiration"]);
            created         = Utils::Time::fromString(j["created"]);

            info = j.at("info");

            state = State::Read;
        }
        else
        {
            return logReadError("license version unknown");
        }
    }
    catch(const std::exception& e)
    {
        return logReadError(e.what());
    }
    catch(...)
    {
        return logReadError("unknown error");
    }
    
    return true;
}

/**
*/
bool License::read(const std::string& license_key,
                   const std::string* expected_id)
{
    *this = {};

    auto logReadError = [ & ] (const std::string& err)
    {
        state = State::ReadError;
        error = err;
        return false;
    };

    auto j = License::unpackageLicenseKey(license_key);
    if (!j.has_value())
        return logReadError("key expansion failed");

    return read(j.value(), expected_id);
}

namespace
{
    EVP_PKEY* load_public_key(const std::string& public_key_str) 
    {
        BIO* bio = BIO_new_mem_buf(public_key_str.data(), -1);
        if (!bio)
            return nullptr;

        EVP_PKEY* pubkey = PEM_read_bio_PUBKEY(bio, nullptr, nullptr, nullptr);
        BIO_free(bio);

        return pubkey;
    }
}

/**
*/
std::pair<bool, std::string> License::verify() const
{
#if USE_EXPERIMENTAL_SOURCE == true

    //get needed bytes
    auto bytes_data      = License::licenseBytes(json_blob);
    auto bytes_signature = License::signatureBytes(signature);

    if (bytes_data.empty())
        return std::make_pair(false, "invalid verification data bytes");
    if (bytes_signature.empty())
        return std::make_pair(false, "invalid verification signature bytes");
    
    // Load the public key
    EVP_PKEY* pubkey = load_public_key(ExtraResources::VerificationKey);
    if (!pubkey)
        return std::make_pair(false, "could not load key");
    
    // Create the context for verification
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx) 
    {
        EVP_PKEY_free(pubkey);
        return std::make_pair(false, "error creating EVP_MD_CTX");
    }

    // Initialize the verification context with SHA-256 and PSS padding
    if (EVP_DigestVerifyInit(ctx, nullptr, EVP_sha256(), nullptr, pubkey) != 1) 
    {
        EVP_MD_CTX_free(ctx);
        EVP_PKEY_free(pubkey);
        return std::make_pair(false, "error initializing DigestVerify");
    }

    // Set up the PSS padding with MGF1 and maximum salt length
    if (EVP_PKEY_CTX_set_rsa_padding(EVP_MD_CTX_pkey_ctx(ctx), RSA_PKCS1_PSS_PADDING) <= 0) 
    {
        EVP_MD_CTX_free(ctx);
        EVP_PKEY_free(pubkey);
        return std::make_pair(false, "error setting RSA PSS padding");
    }

    // -2 sets salt length according to block structure
    if (EVP_PKEY_CTX_set_rsa_pss_saltlen(EVP_MD_CTX_pkey_ctx(ctx), -2) <= 0) 
    {  
        EVP_MD_CTX_free(ctx);
        EVP_PKEY_free(pubkey);
        return std::make_pair(false, "error setting PSS salt length");
    }

    // Verify the data (hash it and prepare for signature verification)
    if (EVP_DigestVerifyUpdate(ctx, bytes_data.data(), bytes_data.size()) != 1) 
    {
        EVP_MD_CTX_free(ctx);
        EVP_PKEY_free(pubkey);
        return std::make_pair(false, "error updating DigestVerify");
    }

    // Finalize the verification (checks the signature)
    int verification_status = EVP_DigestVerifyFinal(ctx, bytes_signature.data(), bytes_signature.size());

    // Clean up
    EVP_MD_CTX_free(ctx);
    EVP_PKEY_free(pubkey);

    if (verification_status == 0)
        return std::make_pair(false, "license invalid");
    else if (verification_status == 1)
        return std::make_pair(true, "");

    return std::make_pair(false, "error during DigestVerifyFinal");

#else
    return std::make_pair(false, "public key missing");
#endif
}

/**
*/
std::pair<License::Validity, std::string> License::validity() const
{
    if (state == State::Empty)
        return std::make_pair(Validity::Empty, "license empty");
    else if (state == State::ReadError)
        return std::make_pair(Validity::ReadError, "license could not be read");
    else if (!isComplete())
        return std::make_pair(Validity::Incomplete, "license incomplete");
    
    auto vres = verify();
    if (!vres.first)
        return std::make_pair(Validity::Invalid, vres.second);

    auto t = Utils::Time::currentUTCTime();

    if (t < date_activation - D0)
        return std::make_pair(Validity::NotActivated, "license not activated");
    if (t > date_expiration + D1)
        return std::make_pair(Validity::Expired, "license expired");

    return std::make_pair(Validity::Valid, "");
}

/**
*/
bool License::componentEnabled(Component c) const
{
    return (type != Type::Free && components.count(c) > 0);
}

/**
*/
std::string License::componentsAsString() const
{
    std::string component_str;
    for (const auto& c : components)
        component_str += (component_str.empty() ? "" : ", ") + componentToString(c);

    return component_str;
}

/**
*/
void License::print() const
{
    loginf << "version:    " << version;
    loginf << "type:       " << typeToString(type);
    loginf << "id:         " << id;
    loginf << "components: " << componentsAsString();
    loginf << "licensee:   " << licensee;
    loginf << "activates:  " << Utils::Time::toDateString(date_activation);
    loginf << "expires:    " << Utils::Time::toDateString(date_expiration);
    loginf << "created:    " << Utils::Time::toString(created);
    loginf << "info:       " << info;
    loginf << "signature:  " << signature;
    loginf << "state:      " << (int)state;
    loginf << "error:      " << error;
}

}
