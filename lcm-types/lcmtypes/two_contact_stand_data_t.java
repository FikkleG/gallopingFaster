/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class two_contact_stand_data_t implements lcm.lcm.LCMEncodable
{
    public double exit_flag;
    public double nWSR;
    public double cpu_time_microseconds;
    public double f_opt[];
    public double f_ref[];
    public double f_control[];
    public double f_unc[];
    public double minForces[];
    public double maxForces[];
    public double contact_state[];
    public double stance_legs;
    public double p_des[];
    public double p_act[];
    public double rpy[];
    public double rpy_act[];
    public double lbA[];
    public double ubA[];
    public double C_times_f[];
    public double s[];
    public double cost_to_go;
    public double Q_lqr[];
    public double R_lqr;
    public double R_fil;
 
    public two_contact_stand_data_t()
    {
        f_opt = new double[12];
        f_ref = new double[4];
        f_control = new double[12];
        f_unc = new double[12];
        minForces = new double[4];
        maxForces = new double[4];
        contact_state = new double[4];
        p_des = new double[3];
        p_act = new double[3];
        rpy = new double[3];
        rpy_act = new double[3];
        lbA = new double[20];
        ubA = new double[20];
        C_times_f = new double[20];
        s = new double[12];
        Q_lqr = new double[12];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x1d23a52a90d0daacL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.two_contact_stand_data_t.class))
            return 0L;
 
        classes.add(lcmtypes.two_contact_stand_data_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeDouble(this.exit_flag); 
 
        outs.writeDouble(this.nWSR); 
 
        outs.writeDouble(this.cpu_time_microseconds); 
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.f_opt[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeDouble(this.f_ref[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.f_control[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.f_unc[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeDouble(this.minForces[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeDouble(this.maxForces[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeDouble(this.contact_state[a]); 
        }
 
        outs.writeDouble(this.stance_legs); 
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.p_des[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.p_act[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.rpy[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.rpy_act[a]); 
        }
 
        for (int a = 0; a < 20; a++) {
            outs.writeDouble(this.lbA[a]); 
        }
 
        for (int a = 0; a < 20; a++) {
            outs.writeDouble(this.ubA[a]); 
        }
 
        for (int a = 0; a < 20; a++) {
            outs.writeDouble(this.C_times_f[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.s[a]); 
        }
 
        outs.writeDouble(this.cost_to_go); 
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.Q_lqr[a]); 
        }
 
        outs.writeDouble(this.R_lqr); 
 
        outs.writeDouble(this.R_fil); 
 
    }
 
    public two_contact_stand_data_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public two_contact_stand_data_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.two_contact_stand_data_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.two_contact_stand_data_t o = new lcmtypes.two_contact_stand_data_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.exit_flag = ins.readDouble();
 
        this.nWSR = ins.readDouble();
 
        this.cpu_time_microseconds = ins.readDouble();
 
        this.f_opt = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.f_opt[a] = ins.readDouble();
        }
 
        this.f_ref = new double[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.f_ref[a] = ins.readDouble();
        }
 
        this.f_control = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.f_control[a] = ins.readDouble();
        }
 
        this.f_unc = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.f_unc[a] = ins.readDouble();
        }
 
        this.minForces = new double[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.minForces[a] = ins.readDouble();
        }
 
        this.maxForces = new double[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.maxForces[a] = ins.readDouble();
        }
 
        this.contact_state = new double[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.contact_state[a] = ins.readDouble();
        }
 
        this.stance_legs = ins.readDouble();
 
        this.p_des = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.p_des[a] = ins.readDouble();
        }
 
        this.p_act = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.p_act[a] = ins.readDouble();
        }
 
        this.rpy = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.rpy[a] = ins.readDouble();
        }
 
        this.rpy_act = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.rpy_act[a] = ins.readDouble();
        }
 
        this.lbA = new double[(int) 20];
        for (int a = 0; a < 20; a++) {
            this.lbA[a] = ins.readDouble();
        }
 
        this.ubA = new double[(int) 20];
        for (int a = 0; a < 20; a++) {
            this.ubA[a] = ins.readDouble();
        }
 
        this.C_times_f = new double[(int) 20];
        for (int a = 0; a < 20; a++) {
            this.C_times_f[a] = ins.readDouble();
        }
 
        this.s = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.s[a] = ins.readDouble();
        }
 
        this.cost_to_go = ins.readDouble();
 
        this.Q_lqr = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.Q_lqr[a] = ins.readDouble();
        }
 
        this.R_lqr = ins.readDouble();
 
        this.R_fil = ins.readDouble();
 
    }
 
    public lcmtypes.two_contact_stand_data_t copy()
    {
        lcmtypes.two_contact_stand_data_t outobj = new lcmtypes.two_contact_stand_data_t();
        outobj.exit_flag = this.exit_flag;
 
        outobj.nWSR = this.nWSR;
 
        outobj.cpu_time_microseconds = this.cpu_time_microseconds;
 
        outobj.f_opt = new double[(int) 12];
        System.arraycopy(this.f_opt, 0, outobj.f_opt, 0, 12); 
        outobj.f_ref = new double[(int) 4];
        System.arraycopy(this.f_ref, 0, outobj.f_ref, 0, 4); 
        outobj.f_control = new double[(int) 12];
        System.arraycopy(this.f_control, 0, outobj.f_control, 0, 12); 
        outobj.f_unc = new double[(int) 12];
        System.arraycopy(this.f_unc, 0, outobj.f_unc, 0, 12); 
        outobj.minForces = new double[(int) 4];
        System.arraycopy(this.minForces, 0, outobj.minForces, 0, 4); 
        outobj.maxForces = new double[(int) 4];
        System.arraycopy(this.maxForces, 0, outobj.maxForces, 0, 4); 
        outobj.contact_state = new double[(int) 4];
        System.arraycopy(this.contact_state, 0, outobj.contact_state, 0, 4); 
        outobj.stance_legs = this.stance_legs;
 
        outobj.p_des = new double[(int) 3];
        System.arraycopy(this.p_des, 0, outobj.p_des, 0, 3); 
        outobj.p_act = new double[(int) 3];
        System.arraycopy(this.p_act, 0, outobj.p_act, 0, 3); 
        outobj.rpy = new double[(int) 3];
        System.arraycopy(this.rpy, 0, outobj.rpy, 0, 3); 
        outobj.rpy_act = new double[(int) 3];
        System.arraycopy(this.rpy_act, 0, outobj.rpy_act, 0, 3); 
        outobj.lbA = new double[(int) 20];
        System.arraycopy(this.lbA, 0, outobj.lbA, 0, 20); 
        outobj.ubA = new double[(int) 20];
        System.arraycopy(this.ubA, 0, outobj.ubA, 0, 20); 
        outobj.C_times_f = new double[(int) 20];
        System.arraycopy(this.C_times_f, 0, outobj.C_times_f, 0, 20); 
        outobj.s = new double[(int) 12];
        System.arraycopy(this.s, 0, outobj.s, 0, 12); 
        outobj.cost_to_go = this.cost_to_go;
 
        outobj.Q_lqr = new double[(int) 12];
        System.arraycopy(this.Q_lqr, 0, outobj.Q_lqr, 0, 12); 
        outobj.R_lqr = this.R_lqr;
 
        outobj.R_fil = this.R_fil;
 
        return outobj;
    }
 
}

