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

/**
 * @brief Base class for singleton pattern
 *
 * Derived classes must override getInstance ().
 */
class Singleton
{
  public:
    /// @brief Returns static instance
    static Singleton& getInstance()
    {
        static Singleton instance;
        return instance;
    }

  protected:
    /// @brief Constructor
    Singleton(){};

  private:
    /// @brief Don't implement
    Singleton(Singleton const&);
    /// @brief Don't implement
    void operator=(Singleton const&);
};
