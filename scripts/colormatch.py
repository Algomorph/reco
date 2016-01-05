import cv2
import numpy as np
import os
import os.path as osp

def calc_pdf(channel):
    numbins = 256
    hist = cv2.calcHist([channel],[0],None,[numbins],(0.,256.))
    pdf_hist = hist / channel.size
    return pdf_hist

def calc_pdf_video(video_path):
    cap = cv2.VideoCapture(video_path)
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    px_count = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) * cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    numbins = 256
    hists = np.zeros((3,numbins),np.float64)
    ret = True
    while(ret == True):
        ret, frame = cap.read()
        channels = cv2.split(frame)
        if(ret):
            hists[0,:] += cv2.calcHist([channels[0]],[0],None,[numbins],(0.,256.)).flatten()
            hists[1,:] += cv2.calcHist([channels[1]],[0],None,[numbins],(0.,256.)).flatten()
            hists[2,:] += cv2.calcHist([channels[2]],[0],None,[numbins],(0.,256.)).flatten()
    hists /= (n_frames * px_count)
    cap.release()
    return hists

def calc_and_save_video_pdfs(work_dir):
    paths = [osp.join(work_dir,path) for path in os.listdir(work_dir) if path.endswith(".mp4")]
    paths.sort()
    hists = []
    for path in paths:
        hists.append(calc_pdf_video(path))
    hists = np.vstack(hists)
    np.save(osp.join(work_dir,"video_channel_pdfs.npy"),hists)
    return hists


def calc_cdf(channel, pdf_hist = None):
    numbins = 256
    #PDF of the reference image (i.e. fractions of all pixels in each bin)
    if(pdf_hist is None):
        pdf_hist = calc_pdf(channel)
    #G = (L-1)*CDF(p)
    acc_cdf = pdf_hist.cumsum()
    cdf = np.round((numbins-1)*acc_cdf)
    return cdf

def match_histogram_channel(source, reference):
    g = calc_cdf(reference)
    s = calc_cdf(source)
    f = np.zeros(s.shape,np.uint8)
    for ix_row in xrange(0,s.shape[0]):
        f[ix_row] = np.argmin(np.abs(-g + s[ix_row]))
        
    return f[source]

def match_histogram_bgr(source, reference):
    target = np.zeros_like(source)
    target[:,:,0] = match_histogram_channel(source[:,:,0], reference[:,:,0]) 
    target[:,:,1] = match_histogram_channel(source[:,:,1], reference[:,:,1])
    target[:,:,2] = match_histogram_channel(source[:,:,2], reference[:,:,2])
    return target

def compute_match_score(source, reference):
    target = match_histogram_bgr(source, reference)
    return 1.0 - np.abs((source.astype(np.int32) - target)).mean() / 255.0

def match_table(images):
    scores = np.ones((len(images),len(images)),np.float64)
    for i_image in range(0,len(images)):
        for j_image in range(i_image, len(images)):
            if(i_image != j_image):
                scores[i_image,j_image] = compute_match_score(images[i_image], images[j_image])
    return np.argsort(scores, 0), scores

def match_table_sets(images, num_cameras):
    set_size = len(images) / num_cameras
    sets = []
    for i_set in range(0,num_cameras):
        sets.append(images[set_size*i_set:set_size*(i_set+1)])
    scores = np.ones((num_cameras,num_cameras),np.float64)
    for i_cam in range(0,num_cameras):
        cur_set = sets[i_cam]
        for j_cam in range(i_cam+1,num_cameras):
            other_set = sets[j_cam]
            cur_scores = []
            for i_im in cur_set:
                for j_im in other_set:
                    cur_scores.append(compute_match_score(i_im, j_im))
            score = np.mean(cur_scores)
            scores[i_cam,j_cam] = score
            scores[j_cam,i_cam] = score
    return np.flipud(np.argsort(scores,0)),scores

def compute_pdf_score(pdf_arr,i_camera, j_camera, n_channels = 3):
    diff = (pdf_arr[n_channels*j_camera:n_channels*j_camera+n_channels, :] - 
            pdf_arr[n_channels*i_camera:n_channels*i_camera+n_channels, :])
    return -np.sum(np.abs(diff))

def match_table_pdf(pdfs, n_channels = 3):
    n_cameras = int(pdfs.shape[0] / n_channels);
    scores = np.zeros((n_cameras,n_cameras),np.float64)
    for i_image in range(0,n_cameras):
        for j_image in range(i_image,n_cameras):
            if(i_image != j_image):
                scores[i_image,j_image] = compute_pdf_score(pdfs, i_image, j_image,n_channels)
    return scores

def best_score_rec(scores, combo, selected, last_score, num_cams, level, num_levels):
    best_score = last_score
    best_combo = combo
    combos_examined = 0
    row_selection = selected.copy()
    if(level < num_levels):
        for i_cam in range(0, num_cams):
            if(i_cam not in selected):
                row_selection.add(i_cam)
                for j_cam in range(i_cam+1, num_cams):
                    if(j_cam not in selected):
                        new_selection = row_selection.copy()
                        new_selection.add(j_cam)
                        new_score = last_score + scores[i_cam,j_cam]
                        score, new_combo, nce = best_score_rec(scores, combo + [(i_cam,j_cam)], new_selection, new_score, num_cams, level+1, num_levels)
                        combos_examined += nce
                        if(score > best_score):
                            best_score = score
                            best_combo = new_combo
    else:
        combos_examined = 1
    return best_score, best_combo, combos_examined    
    
                
def best_pairs(scores, score_func = None):
    processed_scores = scores
    #apply score function
    if(score_func != None):
        process = np.vectorize(score_func)
        processed_scores = process(scores)
    #make sure scores are positive
    if processed_scores.min() < 0:
        processed_scores = processed_scores - processed_scores.min()    
    num_cams = len(scores)
    num_pairs = num_cams / 2
    return best_score_rec(processed_scores, [], set(), 0.0, num_cams, 0, num_pairs)
                
        
        
    
        
            
    
                
                