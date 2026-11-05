"""Shared numeric calculations; plotting is in the individual plot_*.py files."""
import numpy as np

G=9.7936174
OMEGA=7.292115e-5
LAT=np.deg2rad(30.531651244)
DT=0.005
QGD=0.1/(3600*256)
QA=0.05/2**15

def read(text,align=False):
    rows=[]; skipped=0; bad=0; crc_bad=0
    # CRC32 is the NovAtel reflected polynomial with initial zero, no final XOR.
    import zlib
    for line in text.splitlines():
        if not line.startswith('#RAWIMUXA,' if align else '%RAWIMUSA,'):
            skipped+=1; continue
        try:
            body,crc=line.split('*',1)
            calc=zlib.crc32(body[1:].encode('ascii'),0xffffffff)^0xffffffff
            crc_bad+=int(calc != int(crc.strip(),16))
            f=body.split(';',1)[1].split(',')
            if len(f)!=(11 if align else 9): raise ValueError('field count')
            j=2 if align else 0
            rows.append([float(f[j]),float(f[j+1]),int(f[j+2],16),*[int(x) for x in f[j+3:]]])
        except (ValueError,IndexError): bad+=1
    a=np.asarray(rows,dtype=float)
    t=a[:,1]+(a[:,0]-a[0,0])*604800
    ac=a[:,[5,4,3]]*np.array([1,-1,1])
    gy=a[:,[8,7,6]]*np.array([1,-1,1])
    f=ac*(2e-8 if align else QA)/DT
    w=gy*(1e-9 if align else np.deg2rad(QGD))/DT
    delta=np.diff(t)
    quality={'n':len(a),'span_s':float(t[-1]-t[0]),'start_sow':float(t[0]),'end_sow':float(t[-1]),
             'median_dt_s':float(np.median(delta)),'max_dt_s':float(delta.max()),
             'nonpositive_dt':int(np.sum(delta<=0)), 'gaps':int(np.sum(delta>1.5*DT)),
             'bad_records':bad,'other_lines':skipped,'crc_bad':crc_bad,
             'status_words':[hex(int(x)) for x in np.unique(a[:,2])]}
    return t,f,w,quality

def blocks(t,values,seconds=1):
    ids=np.floor((t-t[0]+1e-7)/seconds).astype(int)
    counts=np.bincount(ids)
    sums=np.column_stack([np.bincount(ids,weights=values[:,i]) for i in range(values.shape[1])])
    good=(counts>=int(seconds/DT)-2)
    return (np.flatnonzero(good)+0.5)*seconds,sums[good]/counts[good,None],counts[good]


def calculate(raw):
    result={'method':{'fs':200,'static_trim_each_end_s':5,'rotation_threshold_deg_s':0.1,
                      'rotation_smoothing_samples':51,'rotation_margin_each_side_s':1,
                      'body_from_device':np.eye(3).tolist(), 'gyro_fsas_deg_per_count':QGD},'images':{}}

    data={name:read(text,'RAWIMUXA' in text[:30]) for name,text in raw.items()}
    del raw
    result['data_quality']={name:item[3] for name,item in data.items()}
    names=[f'{a}_{d}.ASC' for a in 'XYZ' for d in ['up','down']]
    means=[]; wm=[]; selected=[]
    for name in names:
        t,f,w,q=data[name]
        mask=(t>=t[0]+5)&(t<=t[-1]-5)
        means.append(f[mask].mean(axis=0));wm.append(w[mask].mean(axis=0));selected.append(mask)
    means=np.array(means);wm=np.array(wm)
    truth=np.repeat(np.eye(3),2,axis=0)*np.tile([G,-G],3)[:,None]
    design=np.column_stack([truth,np.ones(6)])
    coef=np.linalg.lstsq(design,means,rcond=None)[0]
    A=coef[:3].T;b=coef[3];inv=np.linalg.inv(A)
    comp=(means-b)@inv.T
    rms_raw=[];rms_com=[];noise_raw=[];noise_com=[]
    for name,mask,ref in zip(names,selected,truth):
        f=data[name][1][mask];c=(f-b)@inv.T
        rms_raw.append(np.sqrt(np.mean((f-ref)**2,axis=0)))
        rms_com.append(np.sqrt(np.mean((c-ref)**2,axis=0)))
        noise_raw.append(f.std(axis=0,ddof=1));noise_com.append(c.std(axis=0,ddof=1))
    result['accelerometer']={'bias_m_s2':b.tolist(),'scale_ppm':((np.diag(A)-1)*1e6).tolist(),
     'cross_ppm':((A-np.diag(np.diag(A)))*1e6).tolist(),'matrix':A.tolist(),'condition':float(np.linalg.cond(A)),
     'mean_raw':means.tolist(),'mean_compensated':comp.tolist(),'reference':truth.tolist(),
     'selected_n':[int(m.sum()) for m in selected], 'rms_raw':np.array(rms_raw).tolist(),
     'rms_compensated':np.array(rms_com).tolist(),'std_raw':np.array(noise_raw).tolist(),
     'std_compensated':np.array(noise_com).tolist(),
     'mean_error_rms_raw':np.sqrt(np.mean((means-truth)**2,axis=0)).tolist(),
     'mean_error_rms_compensated':np.sqrt(np.mean((comp-truth)**2,axis=0)).tolist()}

    bg=(wm[::2]+wm[1::2]).diagonal()/2
    rot=[];sg=[];windows=[]
    for axis in range(3):
        pair=[]
        for sign in [1,-1]:
            name=f'{"XYZ"[axis]}_down_{"360" if sign==1 else "-360"}.ASC'
            t,f,w,q=data[name];rate=np.rad2deg(w[:,axis])
            baseline=np.mean(rate[:200])
            sm=np.convolve(rate-baseline,np.ones(51)/51,mode='same')
            active=np.flatnonzero(np.abs(sm)>.1)
            if not len(active): raise ValueError('No rotation: '+name)
            pair.append((name,t,w,rate,int(active[0]),int(active[-1])))
        length=max(p[5]-p[4]+1 for p in pair)+400
        angles=[]
        for j,(name,t,w,rate,i0,i1) in enumerate(pair):
            start=max(0,min(len(t)-length,(i0+i1-length)//2))
            stop=start+length
            if start>i0-100 or stop<i1+100: raise ValueError('Insufficient stationary margin: '+name)
            theta=float(np.sum(rate[start:stop])*DT)
            angles.append(theta)
            row={'file':name,'active_start_relative_s':float(t[i0]-t[0]),'active_end_relative_s':float(t[i1]-t[0]),
                 'start_sow':float(t[start]),'end_sow':float(t[stop-1]),'n':length,'T_s':length*DT,
                 'raw_angle_deg':theta,'peak_deg_s':float(np.max(np.abs(rate)))}
            rot.append(row);windows.append((axis,j,name,start,stop))
        sg.append((angles[0]-angles[1])/720-1)
    sg=np.array(sg)
    for row,window in zip(rot,windows):
        axis,j,name,start,stop=window
        # Correct the sensor response first, then remove the downward Earth projection.
        row['comp_angle_deg']=(row['raw_angle_deg']-np.rad2deg(bg[axis])*row['T_s'])/(1+sg[axis])+np.rad2deg(OMEGA*np.sin(LAT))*row['T_s']
        row['reference_deg']=360*(1 if j==0 else -1)
    result['gyroscope']={'bias_deg_h':(np.rad2deg(bg)*3600).tolist(),'scale_ppm':(sg*1e6).tolist(),'windows':rot,
     'static_means_deg_h':(np.rad2deg(wm)*3600).tolist()}

    align_name=next(n for n in data if n.startswith('Received'))
    t,f,w,q=data[align_name]
    def attitude(ff,ww,return_matrix=False):
        ff=np.atleast_2d(ff);ww=np.atleast_2d(ww)
        down=-ff/np.linalg.norm(ff,axis=1)[:,None]
        east=np.cross(down,ww);east/=np.linalg.norm(east,axis=1)[:,None]
        north=np.cross(east,down)
        C=np.stack([north,east,down],axis=1)
        yaw=np.arctan2(C[:,1,0],C[:,0,0])
        pitch=np.arctan2(-C[:,2,0],np.hypot(C[:,2,1],C[:,2,2]))
        roll=np.arctan2(C[:,2,1],C[:,2,2])
        att=np.rad2deg(np.column_stack([yaw,pitch,roll]))
        return (att,C) if return_matrix else att
    whole,C=attitude(f.mean(axis=0),w.mean(axis=0),True);whole=whole[0];C=C[0]
    epoch=attitude(f,w)
    tb,mean6,counts=blocks(t,np.column_stack([f,w]))
    second=attitude(mean6[:,:3],mean6[:,3:])
    def wrap(a): return (a+180)%360-180
    def summary(a):
        e=wrap(a-whole)
        return {'n':len(a),'rms_difference_deg':np.sqrt(np.mean(e**2,axis=0)).tolist(),
                'max_abs_difference_deg':np.max(np.abs(e),axis=0).tolist()}
    convergence=[]
    for sec in [1,2,5,10,30,60,120,300,600,900,1200,1800]:
        m=t<t[0]+sec
        val=attitude(f[m].mean(axis=0),w[m].mean(axis=0))[0]
        convergence.append({'T_s':sec,'angles_deg':val.tolist(),'difference_deg':wrap(val-whole).tolist()})
    result['alignment']={'file':align_name,'whole_yaw_pitch_roll_deg':whole.tolist(),'C_body_to_NED':C.tolist(),
     'mean_force':f.mean(axis=0).tolist(),'mean_omega':w.mean(axis=0).tolist(),
     'orthogonality_frobenius':float(np.linalg.norm(C@C.T-np.eye(3))), 'determinant':float(np.linalg.det(C)),
     'epoch':summary(epoch),'second':summary(second),'convergence':convergence,
     'one_second_n_min':int(counts.min()),'one_second_n_max':int(counts.max()),
     'sigma_heading_bias_deg':float(.05/(np.rad2deg(OMEGA)*3600*np.cos(LAT))*180/np.pi),
     'sigma_heading_arw_1800_deg':float((.005/np.sqrt(.5))/(np.rad2deg(OMEGA)*3600*np.cos(LAT))*180/np.pi)}
    assert all(q['bad_records']==0 and q['crc_bad']==0 and q['nonpositive_dt']==0 for q in result['data_quality'].values())
    assert np.all(np.abs(np.diag(A)-1)<.02)
    assert np.all(np.abs(sg)<.02)
    assert np.linalg.norm(C@C.T-np.eye(3))<1e-12 and abs(np.linalg.det(C)-1)<1e-12
    assert np.all(np.isfinite(epoch))

    return locals()

