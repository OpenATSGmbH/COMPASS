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

#pragma once

#include "propertylist.h"
#include "logger.h"
#include "traced_assert.h"

#include "json_fwd.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>


namespace dbContent {

class VariableSet;

}

template <class T>
class NullableVector;

typedef std::tuple<std::map<std::string, std::shared_ptr<NullableVector<bool>>>,
std::map<std::string, std::shared_ptr<NullableVector<char>>>,
std::map<std::string, std::shared_ptr<NullableVector<unsigned char>>>,
std::map<std::string, std::shared_ptr<NullableVector<int>>>,
std::map<std::string, std::shared_ptr<NullableVector<unsigned int>>>,
std::map<std::string, std::shared_ptr<NullableVector<long int>>>,
std::map<std::string, std::shared_ptr<NullableVector<unsigned long int>>>,
std::map<std::string, std::shared_ptr<NullableVector<float>>>,
std::map<std::string, std::shared_ptr<NullableVector<double>>>,
std::map<std::string, std::shared_ptr<NullableVector<std::string>>>,
std::map<std::string, std::shared_ptr<NullableVector<nlohmann::json>>>,
std::map<std::string, std::shared_ptr<NullableVector<boost::posix_time::ptime>>>>
ArrayListMapTupel;

template <class T, class Tuple>
struct BufferIndex;

template <class T, class... Types>
struct BufferIndex<T, std::tuple<T, Types...>>
{
    static const std::size_t value = 0;
};

template <class T, class U, class... Types>
struct BufferIndex<T, std::tuple<U, Types...>>
{
    static const std::size_t value = 1 + BufferIndex<T, std::tuple<Types...>>::value;
};

class Buffer
{
    template <class T>
    friend class NullableVector;
    //friend class OldNullableVector;

public:
    Buffer(PropertyList properties, const std::string& dbcontent_name = "");
    virtual ~Buffer();

    // Adds all containers of org_buffer and removes them from org_buffer.
    void seizeBuffer(Buffer& org_buffer);

    bool hasProperty(const Property& property);
    bool hasAnyPropertyNamed (const std::string& property_name);
    void addProperty(std::string id, PropertyDataType type);
    void addProperty(const Property& property);
    void deleteProperty(const Property& property);

    const PropertyList& properties() const;
    void printProperties();

    void sortByProperty(const Property& property);

    template <typename T>
    bool has(const std::string& id) const;

    template <typename T>
    NullableVector<T>& get(const std::string& id);
    template <typename T>
    const NullableVector<T>& get(const std::string& id) const;

    template <typename T>
    void rename(const std::string& id, const std::string& id_new);

    // Returns current size
    size_t size() const;
    void cutToSize(size_t size);

    void cutUpToIndex(size_t index); // everything up to index is removed
    void removeIndexes(const std::vector<unsigned int>& indexes_to_remove); // must be sorted

    const std::string& dbContentName() const { return dbcontent_name_; }

    void dbContentName(const std::string& dbcontent_name) { dbcontent_name_ = dbcontent_name; }

    bool isNull(const Property& property, unsigned int index);
    void deleteEmptyProperties();

    void transformVariables(dbContent::VariableSet& list,
                            bool dbcol2dbcontvar);  // tc2dbcontvar true for db col -> dbcont var, false dbcont var -> db column

    nlohmann::json asJSON(unsigned int max_size=0);
    nlohmann::json asJSON(std::set<std::string> variable_names, unsigned int max_size=0);

protected:
    PropertyList properties_;
    std::string dbcontent_name_;

    ArrayListMapTupel array_list_tuple_;
    size_t size_ {0};

private:
    template <typename T>
    inline std::map<std::string, std::shared_ptr<NullableVector<T>>>& getArrayListMap();
    template <typename T>
    inline const std::map<std::string, std::shared_ptr<NullableVector<T>>>& getArrayListMap() const;
    template <typename T>
    void renameArrayListMapEntry(const std::string& id, const std::string& id_new);
    template <typename T>
    void seizeArrayListMap(Buffer& org_buffer);

    template <typename T>
    void remove(const std::string& id);
};

#include "nullablevector.h"

template <typename T>
inline bool Buffer::has(const std::string& id) const
{
    return getArrayListMap<T>().count(id) != 0;
}

template <typename T>
NullableVector<T>& Buffer::get(const std::string& id)
{
    if (!(std::get<BufferIndex<std::map<std::string, std::shared_ptr<NullableVector<T>>>,
          ArrayListMapTupel>::value>(array_list_tuple_)).count(id))
        logerr << "id '" << id << "' type " << typeid(T).name() << " not found";

    traced_assert((std::get<BufferIndex<std::map<std::string, std::shared_ptr<NullableVector<T>>>,
             ArrayListMapTupel>::value>(array_list_tuple_)).count(id));

    return *(std::get<BufferIndex<std::map<std::string, std::shared_ptr<NullableVector<T>>>,
             ArrayListMapTupel>::value>(array_list_tuple_))
            .at(id);
}

template <typename T>
const NullableVector<T>& Buffer::get(const std::string& id) const
{
    if (!(std::get<BufferIndex<std::map<std::string, std::shared_ptr<NullableVector<T>>>,
          ArrayListMapTupel>::value>(array_list_tuple_)).count(id))
        logerr << "id '" << id << "' type " << typeid(T).name() << " not found";

    traced_assert((std::get<BufferIndex<std::map<std::string, std::shared_ptr<NullableVector<T>>>,
             ArrayListMapTupel>::value>(array_list_tuple_)).count(id));

    return *(std::get<BufferIndex<std::map<std::string, std::shared_ptr<NullableVector<T>>>,
             ArrayListMapTupel>::value>(array_list_tuple_))
            .at(id);
}

template <typename T>
void Buffer::rename(const std::string& id, const std::string& id_new)
{
    renameArrayListMapEntry<T>(id, id_new);

    traced_assert(properties_.hasProperty(id));
    Property old_property = properties_.get(id);
    properties_.removeProperty(id);
    properties_.addProperty(id_new, old_property.dataType());
}

template <typename T>
void Buffer::remove(const std::string& id)
{
    traced_assert(getArrayListMap<T>().count(id) == 1);
    traced_assert(properties_.hasProperty(id));

    getArrayListMap<T>().erase(id);
    properties_.removeProperty(id);
}

// private stuff

template <typename T>
std::map<std::string, std::shared_ptr<NullableVector<T>>>& Buffer::getArrayListMap()
{
    return std::get<
            BufferIndex<std::map<std::string, std::shared_ptr<NullableVector<T>>>, ArrayListMapTupel>::value>(
                array_list_tuple_);
}

template <typename T>
const std::map<std::string, std::shared_ptr<NullableVector<T>>>& Buffer::getArrayListMap() const
{
    return std::get<
            BufferIndex<std::map<std::string, std::shared_ptr<NullableVector<T>>>, ArrayListMapTupel>::value>(
                array_list_tuple_);
}

template <typename T>
void Buffer::renameArrayListMapEntry(const std::string& id, const std::string& id_new)
{
    traced_assert(getArrayListMap<T>().count(id) == 1);
    traced_assert(getArrayListMap<T>().count(id_new) == 0);
    std::shared_ptr<NullableVector<T>> array_list = getArrayListMap<T>().at(id);
    array_list->renameProperty(id_new);
    getArrayListMap<T>().erase(id);
    getArrayListMap<T>()[id_new] = array_list;
}

template <typename T>
void Buffer::seizeArrayListMap(Buffer& other_buffer)
{
    //assert(getArrayListMap<T>().size() == other_buffer.getArrayListMap<T>().size());

    //    loginf << "this properties";
    //    printProperties();
    //    loginf << "other properties";
    //    other_buffer.printProperties();

    // add all properties of other vector
    for(auto& prop_it : other_buffer.properties().properties())
    {
        logdbg << "checking prop name '" << prop_it.name() << "' type "
               << prop_it.dataTypeString() << " contained " << hasProperty(prop_it);

        if (!hasProperty(prop_it))
        {
            logdbg << "adding prop name '" << prop_it.name() << "' type "
                   << prop_it.dataTypeString();
            addProperty(prop_it);
        }
    }

    for (auto& it : other_buffer.getArrayListMap<T>())
    {
        logdbg << "seizing '" << it.first << "'";
        traced_assert(other_buffer.properties().hasProperty(it.first));

        traced_assert(getArrayListMap<T>().count(it.first));
        getArrayListMap<T>().at(it.first)->addData(*it.second);
    }

    other_buffer.getArrayListMap<T>().clear();

    // size_adjusted in seizeBuffer
}

