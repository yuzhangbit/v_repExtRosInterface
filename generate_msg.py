from sys import argv, exit, stderr
from subprocess import check_output
import re

# used to resolve messages specified without a package name
# name -> pkg/name
resolve_msg = {}

def is_identifier(s):
    return re.match('^[a-zA-Z_][a-zA-Z0-9_]*$', s)

# parse a type specification, such as Header, geometry_msgs/Point, or string[12]
class TypeSpec:
    def __init__(self, s):
        self.array = False
        self.array_size = None
        m = re.match(r'^(.*)\[(\d*)\]$', s)
        if m:
            self.array = True
            s = m.group(1)
            if len(m.group(2)) > 0:
                self.array_size = int(m.group(2))
        # perform substitutions:
        if s in resolve_msg: s = resolve_msg[s]
        if s == 'byte': s = 'int8' # deprecated
        if s == 'char': s = 'uint8' # deprecated
        # check builtins:
        self.builtin = s in ('bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64', 'string', 'time', 'duration')
        self.fullname = s
        if self.builtin:
            self.mtype = s
        else:
            if '/' not in s:
                raise ValueError('bad type: %s' % s)
            tok = s.split('/')
            if len(tok) != 2:
                raise ValueError('bad type: %s' % s)
            if not is_identifier(tok[0]) or not is_identifier(tok[1]):
                raise ValueError('bad type: %s' % s)
            self.package = tok[0]
            self.mtype = tok[1]

    # normalize fullname to C identifier (replace / with __)
    def normalized(self):
        return ('{}__'.format(self.package) if not self.builtin else '') + self.mtype

    # get C++ type declaration
    def ctype(self):
        if self.builtin:
            if self.mtype == 'bool': return 'uint8_t'
            if self.mtype == 'int8': return 'int8_t'
            if self.mtype == 'uint8': return 'uint8_t'
            if self.mtype == 'int16': return 'int16_t'
            if self.mtype == 'uint16': return 'uint16_t'
            if self.mtype == 'int32': return 'int32_t'
            if self.mtype == 'uint32': return 'uint32_t'
            if self.mtype == 'int64': return 'int64_t'
            if self.mtype == 'uint64': return 'uint64_t'
            if self.mtype == 'float32': return 'float'
            if self.mtype == 'float64': return 'double'
            if self.mtype == 'string': return 'std::string'
            if self.mtype == 'time': return 'ros::Time'
            if self.mtype == 'duration': return 'ros::Duration'
            raise Exception('can\'t get ctype of builtin %s' % self.mtype)
        return self.package + '::' + self.mtype

    def __str__(self):
        t = self.mtype
        if not self.builtin:
            t = self.package + '/' + t
        if self.array:
            t += '[]'
        return t

# parse message definition
def get_msg_fields(msg_name):
    fields = {}

    out = check_output(['rosmsg', 'show', '-r', msg_name])

    for ln_orig in out.split('\n'):
        ln = ln_orig.strip()

        if '#' in ln:
            # strip comments
            ln = ln[:ln.find('#')].strip()

        if ln == '':
            # ignore empty lines
            continue

        ln_orig1 = ln

        ln = ln.replace('=',' = ')

        tokens = ln.split()

        if len(tokens) == 4 and tokens[2] == '=':
            # it's a constant definition: ignore
            continue

        if len(tokens) == 2:
            t = TypeSpec(tokens[0])
            n = tokens[1]
            fields[n] = t
        else:
            raise ValueError('unrecognized line: ' + ln_orig1)

    return fields

def generate_h(gt, fields, d, f):
    s = '''
bool write__{norm}(const {ctype}& msg, int stack);
bool read__{norm}(int stack, {ctype} *msg);
void ros_callback__{norm}(const boost::shared_ptr<{ctype} const>& msg, SubscriberProxy *proxy);
'''.format(**d)
    f.write(s)

def generate_cpp(gt, fields, d, f):
    wf = '''
bool write__{norm}(const {ctype}& msg, int stack)
{{
    if(simPushTableOntoStack(stack) == -1)
    {{
        std::cerr << "write__{norm}" << ": " << "error: " << "push table failed." << std::endl;
        return false;
    }}'''.format(**d)
    for n, t in fields.items():
        d1 = d
        d1['n'] = n
        d1['t'] = t
        d1['ctype1'] = t.ctype()
        d1['nf'] = '{}::{}'.format(gt.ctype(), n)
        d1['norm1'] = t.normalized()
        if t.array:
            wf += '''
    if(simPushStringOntoStack(stack, "{n}", 0) == -1)
    {{
        std::cerr << "write__{norm}" << ": " << "error: " << "push table key (" << "{nf}" << ") failed." << std::endl;
        return false;
    }}
    if(simPushTableOntoStack(stack) == -1)
    {{
        std::cerr << "write__{norm}" << ": " << "error: " << "push array table (" << "{nf}" << ") failed." << std::endl;
        return false;
    }}
    for(int i = 0; i < msg.{n}.size(); i++)
    {{
        if(!write__int32(i + 1, stack))
        {{
            std::cerr << "write__{norm}" << ": " << "error: " << "push array table key " << i << " (" << "{nf}" << ") failed." << std::endl;
            return false;
        }}
        if(!write__{norm1}(msg.{n}[i], stack))
        {{
            std::cerr << "write__{norm}" << ": " << "error: " << "push array table value (" << "{nf}" << ") failed." << std::endl;
            return false;
        }}
        if(simInsertDataIntoStackTable(stack) == -1)
        {{
            std::cerr << "write__{norm}" << ": " << "error: " << "insert array table pair (" << "{nf}" << ") failed." << std::endl;
            return false;
        }}
    }}
    if(simInsertDataIntoStackTable(stack) == -1)
    {{
        std::cerr << "write__{norm}" << ": " << "error: " << "insert table pair (" << "{nf}" << ") failed." << std::endl;
        return false;
    }}
'''.format(**d1)
        else:
            wf += '''
    if(simPushStringOntoStack(stack, "{n}", 0) == -1)
    {{
        std::cerr << "write__{norm}" << ": " << "error: " << "push table key (" << "{nf}" << ") failed." << std::endl;
        return false;
    }}
    if(!write__{norm1}(msg.{n}, stack))
    {{
        std::cerr << "write__{norm}" << ": " << "error: " << "push table field " << "{nf}" << " of type " << "{t}" << " failed." << std::endl;
        return false;
    }}
    if(simInsertDataIntoStackTable(stack) == -1)
    {{
        std::cerr << "write__{norm}" << ": " << "error: " << "insert table pair " << "{nf}" << " failed." << std::endl;
        return false;
    }}'''.format(**d1)
    wf += '''
    return true;
}}

'''.format(**d)
    f.write(wf)

    rf = '''
bool read__{norm}(int stack, {ctype} *msg)
{{
    int i;
    if((i = simGetStackTableInfo(stack, 0)) != sim_stack_table_map)
    {{
        std::cerr << "read__{norm}" << ": " << "error: " << "expected a table (simGetStackTableInfo returned " << i << ")." << std::endl;
        return false;
    }}

    int sz = simGetStackSize(stack);
    simUnfoldStackTable(stack);
    int numItems = (simGetStackSize(stack) - sz + 1) / 2;

    char *str;
    int strSz;

    while(numItems >= 1)
    {{
        simMoveStackItemToTop(stack, simGetStackSize(stack) - 2); // move key to top
        if((str = simGetStackStringValue(stack, &strSz)) != NULL && strSz > 0)
        {{
            simPopStackItem(stack, 1); // now stack top is value

            if(0) {{}}'''.format(**d)
    for n, t in fields.items():
        d1 = d
        d1['n'] = n
        d1['t'] = t
        d1['ctype1'] = t.ctype()
        d1['nf'] = '{}::{}'.format(gt.ctype(), n)
        d1['norm1'] = t.normalized()
        if t.array:
            if t.array_size:
                ins = 'msg->{n}[i] = (v);'.format(**d1)
            else:
                ins = 'msg->{n}.push_back(v);'.format(**d1)
            rf += '''
            else if(strcmp(str, "{n}") == 0)
            {{
                int i;
                if((i = simGetStackTableInfo(stack, 0)) < 0)
                {{
                    std::cerr << "read__{norm}" << ": " << "error: " << "expected a array-table (simGetStackTableInfo returned " << i << ")." << std::endl;
                    return false;
                }}
                int sz1 = simGetStackSize(stack);
                simUnfoldStackTable(stack);
                int numItems = (simGetStackSize(stack) - sz + 1) / 2;
                for(int i = 0; i < numItems; i++)
                {{
                    simMoveStackItemToTop(stack, simGetStackSize(stack) - 2); // move key to top
                    int j;
                    if(!read__int32(stack, &j))
                    {{
                        std::cerr << "read__{norm}" << ": " << "error: " << "not array table (" << str << ")." << std::endl;
                        return false;
                    }}
                    simPopStackItem(stack, 1); // now stack top is value
                    {ctype1} v;
                    if(!read__{norm1}(stack, &v))
                    {{
                        std::cerr << "read__{norm}" << ": " << "error: " << "value is not " << "{t}" << " for key: " << str << "." << std::endl;
                        return false;
                    }}
                    {ins}
                    simPopStackItem(stack, 1);
                }}
            }}'''.format(ins=ins, **d1)
        else:
            rf += '''
            else if(strcmp(str, "{n}") == 0)
            {{
                if(!read__{norm1}(stack, &(msg->{n})))
                {{
                    std::cerr << "read__{norm}" << ": " << "error: " << "value is not " << "{t}" << " for key: " << str << "." << std::endl;
                    return false;
                }}
            }}'''.format(**d1)
    rf += '''
            else
            {{
                std::cerr << "read__{norm}" << ": " << "error: " << "unexpected key: " << str << "." << std::endl;
                return false;
            }}

            simReleaseBuffer(str);
        }}
        else
        {{
            std::cerr << "read__{norm}" << ": " << "error: " << "malformed table (bad key type)." << std::endl;
            return false;
        }}

        numItems = (simGetStackSize(stack) - sz + 1) / 2;
    }}
    
    return true;
}}

'''.format(**d)
    f.write(rf)

    cb = '''
void ros_callback__{norm}(const boost::shared_ptr<{ctype} const>& msg, SubscriberProxy *proxy)
{{
    int stack = simCreateStack();
    if(stack != -1)
    {{
        do
        {{
            if(!write__{norm}(*msg, stack))
            {{
                break;
            }}
            if(simCallScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack) == -1)
            {{
                std::cerr << "ros_callback__{norm}" << ": " << "error: " << "call script failed." << std::endl;
                break;
            }}
        }}
        while(0);
        simReleaseStack(stack);
    }}
}}

'''.format(**d)
    f.write(cb)

def generate_pub(gt, fields, d, f):
    p = '''    else if(publisherProxy->topicType == "{fn}")
    {{
        {ctype} msg;
        if(!read__{norm}(p->stackID, &msg))
        {{
            simSetLastError("simExtROS_publish", "invalid message format (check stderr)");
            return;
        }}
        publisherProxy->publisher.publish(msg);
    }}
'''.format(**d)
    f.write(p)

def generate_adv(gt, fields, d, f):
    p = '''    else if(in->topicType == "{fn}")
    {{
        publisherProxy->publisher = nh->advertise<{ctype}>(in->topicName, in->queueSize, in->latch);
    }}
'''.format(**d)
    f.write(p)

def generate_sub(gt, fields, d, f):
    p = '''    else if(in->topicType == "{fn}")
    {{
        subscriberProxy->subscriber = nh->subscribe<{ctype}>(in->topicName, in->queueSize, boost::bind(ros_callback__{norm}, _1, subscriberProxy));
    }}
'''.format(**d)
    f.write(p)

def main(argc, argv):
    if argc != 2:
        stderr.write('argument error\n')
        exit(42)

    # populate resolve_msg dictionary
    with open(argv[1]) as f:
        for l in f.readlines():
            l = l.strip()
            pkg, n = l.split('/')
            resolve_msg[n] = l

    f_cpp = open('generated/ros_msg_io.cpp', 'w')
    f_h = open('generated/ros_msg_io.h', 'w')
    f_adv = open('generated/adv.cpp', 'w')
    f_pub = open('generated/pub.cpp', 'w')
    f_sub = open('generated/sub.cpp', 'w')

    f_cpp.write('''#include <ros_msg_builtin_io.h>
#include <ros_msg_io.h>
#include <v_repLib.h>

''')
    f_h.write('''#ifndef VREP_ROS_PLUGIN__ROS_MSG_IO__H
#define VREP_ROS_PLUGIN__ROS_MSG_IO__H

#include <ros/ros.h>
#include <vrep_ros_plugin.h>

''') 

    print('Generating header...')
    # for each msg include msg header
    for msg in resolve_msg.values():
        f_h.write('#include <%s.h>\n' % msg)
    f_h.write('\n')

    # for each msg generate cpp, h, adv, pub, sub code
    for msg in resolve_msg.values():
        print('Generating code for message %s...' % msg)
        gt = TypeSpec(msg)
        fields = get_msg_fields(msg)
        d = {'norm': gt.normalized(), 'ctype': gt.ctype(), 'fn': gt.fullname}
        generate_cpp(gt, fields, d, f_cpp)
        generate_h(gt, fields, d, f_h)
        generate_adv(gt, fields, d, f_adv)
        generate_pub(gt, fields, d, f_pub)
        generate_sub(gt, fields, d, f_sub)

    f_h.write('''

#endif // VREP_ROS_PLUGIN__ROS_MSG_IO__H
''')

    f_cpp.close()
    f_h.close()
    f_adv.close()
    f_pub.close()
    f_sub.close()

if __name__ == '__main__':
    main(len(argv), argv)
