// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <inttypes.h>
#include <memory>
#include "turtlesim_composition/srv/harmonic.hpp"
#include "rclcpp/rclcpp.hpp"
#include <math.h>

#define PI 3.14159265358979323846

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class MinimalServer : public rclcpp::Node
{
public:
  MinimalServer()
  : Node("minimal_server")
  {
    /*init the service*/
  }

private:

  void handle_service(/*   
  
  				*/)
  {
  //callback
  
  }
  rclcpp::Service<turtlesim_composition::srv::Harmonic>::SharedPtr service_
};

int main(int argc, char * argv[])
{
  /* init and 
  spin*/
  
  rclcpp::shutdown();
  return 0;
}
