/*
 * Module Name: qrem_core
 * Author(s):
        York University - Lassonde School of Engineering - 2026 Capstone - Team 32
            - Salwan Aldhahab
            - Jessica Buentipo
            - Mai Komar
            - Kiet Le
            - Quardin Lyttle
            - Mavra Muzmmal
 * Description: Top-level integration of the ML-KEM accelerator.
 *
 * BRING-UP NOTES:
 *   1. TR, HSU, and PAU error outputs are still tied low at the CCU boundary
 *      until those repos expose top-level error ports.
 *   2. HSU row/col/cbd_n and PAU job metadata are now sourced from the CCU's
 *      latched KeyGen sequencing, not hardwired defaults.
 *   3. PAU's 16-bit coefficient interface is adapted explicitly to the
 *      Memory subsystem's 12-bit coefficient-domain contract in this top.
 */

import qrem_global_pkg::*;
import core_ctrl_pkg::*;

module qrem_core (
    input  logic                         clk,
    input  logic                         rst,

    // Host Command Interface
    input  logic                         cmd_valid_i,
    output logic                         cmd_ready_o,
    input  logic [3:0]                   cmd_opcode_i,
    input  logic [3:0]                   cmd_mode_i,
    input  logic [1:0]                   cmd_sec_lvl_i,
    input  logic [4:0]                   cmd_payload_id_i,
    input  logic [15:0]                  cmd_xfer_len_i,
    input  logic                         cmd_zeroize_i,

    // Host Status Interface
    output logic                         sts_busy_o,
    output logic                         sts_done_o,
    output logic [3:0]                   sts_err_code_o,

    // AXI-Stream RX (Host -> Transcoder)
    input  logic [63:0]                  s_axis_tdata_i,
    input  logic                         s_axis_tvalid_i,
    output logic                         s_axis_tready_o,
    input  logic                         s_axis_tlast_i,

    // AXI-Stream TX (Transcoder -> Host)
    output logic [63:0]                  m_axis_tdata_o,
    output logic                         m_axis_tvalid_o,
    input  logic                         m_axis_tready_i,
    output logic [7:0]                   m_axis_tkeep_o,
    output logic                         m_axis_tlast_o
);

    localparam int PAU_WORD_W = 16;

    // =========================================================================
    // Interconnect Wires
    // =========================================================================

    // Transcoder <-> CCU
    logic                           tr_start;
    tr_opcode_t                     tr_opcode;
    logic                           tr_done;

    // HSU <-> CCU
    logic                           hsu_start;
    hs_mode_t                       hsu_mode;
    logic [CTRL_XOF_LEN_W-1:0]      hsu_xof_len;
    logic                           hsu_is_eta3;
    logic [POLY_ID_WIDTH-1:0]       hsu_poly_id;
    seed_id_e                       hsu_seed_id;
    logic [1:0]                     hsu_input_sel;
    logic                           hsu_absorb_poly;
    logic                           hsu_absorb_last;
    logic [7:0]                     hsu_row;
    logic [7:0]                     hsu_col;
    logic [7:0]                     hsu_cbd_n;
    logic                           hsu_done;
    logic                           hsu_packer_done;
    logic                           hsu_hash_ek_read_en;

    // PAU <-> CCU
    logic                           pau_start;
    ctrl_pau_job_t                  pau_job;
    logic                           pau_done;
    logic [POLY_ID_WIDTH-1:0]       pau_poly_id;
    logic [POLY_ID_WIDTH-1:0]       pau_cwm_num_terms;

    // Memory <-> CCU
    logic                           mem_zeroize_req;
    logic                           mem_zeroize_done;
    logic                           mem_fault;
    logic [2:0]                     mem_fault_code;
    ctrl_mem_phase_t                mem_phase; // Sideband debug/intent
    logic [1:0]                     active_sec_lvl;

    // PAU <-> Memory Primary
    logic                           pau_mem_req;
    logic                           pau_mem_rd_en;
    logic [POLY_ID_WIDTH-1:0]       pau_mem_rd_poly_id;
    logic [3:0][7:0]                pau_mem_rd_idx;
    logic [3:0]                     pau_mem_rd_lane_valid;
    logic [3:0]                     pau_mem_wr_en;
    logic [POLY_ID_WIDTH-1:0]       pau_mem_wr_poly_id;
    logic [3:0][7:0]                pau_mem_wr_idx;
    logic [3:0][COEFF_WIDTH-1:0]    pau_mem_wr_data;
    logic                           pau_mem_rd_valid;
    logic [POLY_ID_WIDTH-1:0]       pau_mem_rd_poly_id_out;
    logic [3:0][7:0]                pau_mem_rd_idx_out;
    logic [3:0]                     pau_mem_rd_lane_valid_out;
    logic [3:0][COEFF_WIDTH-1:0]    pau_mem_rd_data;
    logic                           pau_mem_stall;
    logic [3:0][PAU_WORD_W-1:0]     pau_mem_wr_data_pau;
    logic [3:0][PAU_WORD_W-1:0]     pau_mem_rd_data_pau;

    // PAU <-> Memory Auxiliary
    logic                           pau_aux_req;
    logic                           pau_aux_rd_en;
    logic [POLY_ID_WIDTH-1:0]       pau_aux_rd_poly_id;
    logic [3:0][7:0]                pau_aux_rd_idx;
    logic [3:0]                     pau_aux_rd_lane_valid;
    logic [3:0]                     pau_aux_wr_en;
    logic [POLY_ID_WIDTH-1:0]       pau_aux_wr_poly_id;
    logic [3:0][7:0]                pau_aux_wr_idx;
    logic [3:0][COEFF_WIDTH-1:0]    pau_aux_wr_data;
    logic                           pau_aux_rd_valid;
    logic [POLY_ID_WIDTH-1:0]       pau_aux_rd_poly_id_out;
    logic [3:0][7:0]                pau_aux_rd_idx_out;
    logic [3:0]                     pau_aux_rd_lane_valid_out;
    logic [3:0][COEFF_WIDTH-1:0]    pau_aux_rd_data;
    logic [3:0][PAU_WORD_W-1:0]     pau_aux_wr_data_pau;
    logic [3:0][PAU_WORD_W-1:0]     pau_aux_rd_data_pau;

    // HSU <-> Memory (Poly)
    logic                           hsu_mem_req;
    logic                           hsu_mem_rd_en;
    logic [POLY_ID_WIDTH-1:0]       hsu_mem_rd_poly_id;
    logic [3:0][7:0]                hsu_mem_rd_idx;
    logic [3:0]                     hsu_mem_rd_lane_valid;
    logic [3:0]                     hsu_mem_wr_en;
    logic [POLY_ID_WIDTH-1:0]       hsu_mem_wr_poly_id;
    logic [3:0][7:0]                hsu_mem_wr_idx;
    logic [3:0][COEFF_WIDTH-1:0]    hsu_mem_wr_data;
    logic                           hsu_mem_rd_valid;
    logic [POLY_ID_WIDTH-1:0]       hsu_mem_rd_poly_id_out;
    logic [3:0][7:0]                hsu_mem_rd_idx_out;
    logic [3:0]                     hsu_mem_rd_lane_valid_out;
    logic [3:0][COEFF_WIDTH-1:0]    hsu_mem_rd_data;
    logic                           hsu_mem_stall;

    // Transcoder <-> Memory (Poly)
    logic                           tr_mem_req;
    logic                           tr_mem_rd_en;
    logic [POLY_ID_WIDTH-1:0]       tr_mem_rd_poly_id;
    logic [3:0][7:0]                tr_mem_rd_idx;
    logic [3:0]                     tr_mem_rd_lane_valid;
    logic [3:0]                     tr_mem_wr_en;
    logic [POLY_ID_WIDTH-1:0]       tr_mem_wr_poly_id;
    logic [3:0][7:0]                tr_mem_wr_idx;
    logic [3:0][COEFF_WIDTH-1:0]    tr_mem_wr_data;
    logic                           tr_mem_rd_valid;
    logic [POLY_ID_WIDTH-1:0]       tr_mem_rd_poly_id_out;
    logic [3:0][7:0]                tr_mem_rd_idx_out;
    logic [3:0]                     tr_mem_rd_lane_valid_out;
    logic [3:0][COEFF_WIDTH-1:0]    tr_mem_rd_data;
    logic                           tr_mem_stall;

    // HSU <-> Memory (Seed)
    logic                           hsu_seed_req;
    logic                           hsu_seed_we;
    seed_id_e                       hsu_mem_seed_id; // Mapped from HSU to Mem
    logic [$clog2(SEED_BEATS)-1:0]  hsu_seed_idx;
    logic [SEED_W-1:0]              hsu_seed_wdata;
    logic                           hsu_seed_ready;
    logic                           hsu_seed_rvalid;
    logic [SEED_W-1:0]              hsu_seed_rdata;

    // Transcoder <-> Memory (Seed)
    logic                           tr_seed_req;
    logic                           tr_seed_we;
    seed_id_e                       tr_seed_id;
    logic [$clog2(SEED_BEATS)-1:0]  tr_seed_idx;
    logic [SEED_W-1:0]              tr_seed_wdata;
    logic                           tr_seed_ready;
    logic                           tr_seed_rvalid;
    logic [SEED_W-1:0]              tr_seed_rdata;

    // HSU <-> Transcoder (Hash Snoop)
    logic [63:0]                    hash_snoop_data;
    logic [7:0]                     hash_snoop_keep;
    logic                           hash_snoop_valid;
    logic                           hash_snoop_ready;
    logic                           hash_snoop_last;

    genvar pau_lane;
    generate
        for (pau_lane = 0; pau_lane < 4; pau_lane++) begin : g_pau_coeff_glue
            assign pau_mem_wr_data[pau_lane]     = pau_mem_wr_data_pau[pau_lane][COEFF_WIDTH-1:0];
            assign pau_mem_rd_data_pau[pau_lane] = {{(PAU_WORD_W-COEFF_WIDTH){1'b0}}, pau_mem_rd_data[pau_lane]};

            assign pau_aux_wr_data[pau_lane]     = pau_aux_wr_data_pau[pau_lane][COEFF_WIDTH-1:0];
            assign pau_aux_rd_data_pau[pau_lane] = {{(PAU_WORD_W-COEFF_WIDTH){1'b0}}, pau_aux_rd_data[pau_lane]};
        end
    endgenerate


    // =========================================================================
    // Core Control Unit (CCU)
    // =========================================================================
    core_control_unit u_ccu (
        .clk                    (clk),
        .rst                    (rst),

        // Host Cmd
        .cmd_valid_i            (cmd_valid_i),
        .cmd_ready_o            (cmd_ready_o),
        .cmd_opcode_i           (cmd_opcode_i),
        .cmd_mode_i             (cmd_mode_i),
        .cmd_sec_lvl_i          (cmd_sec_lvl_i),
        .cmd_payload_id_i       (cmd_payload_id_i),
        .cmd_xfer_len_i         (cmd_xfer_len_i),
        .cmd_zeroize_i          (cmd_zeroize_i),

        // Host Status
        .sts_busy_o             (sts_busy_o),
        .sts_done_o             (sts_done_o),
        .sts_err_code_o         (sts_err_code_o),

        // Transcoder
        .tr_start_o             (tr_start),
        .tr_opcode_o            (tr_opcode),
        .tr_done_i              (tr_done),
        .tr_err_i               (4'h0), // See documentation header note 1

        // HSU
        .hsu_start_o            (hsu_start),
        .hsu_mode_o             (hsu_mode),
        .hsu_xof_len_o          (hsu_xof_len),
        .hsu_is_eta3_o          (hsu_is_eta3),
        .hsu_poly_id_o          (hsu_poly_id),
        .hsu_seed_id_o          (hsu_seed_id),
        .hsu_input_sel_o        (hsu_input_sel),
        .hsu_absorb_poly_o      (hsu_absorb_poly),
        .hsu_absorb_last_o      (hsu_absorb_last),
        .hsu_row_o              (hsu_row),
        .hsu_col_o              (hsu_col),
        .hsu_cbd_n_o            (hsu_cbd_n),
        .hsu_done_i             (hsu_done),
        .hsu_packer_done_i      (hsu_packer_done),
        .hsu_err_i              (4'h0), // See documentation header note 1

        // PAU
        .pau_start_o            (pau_start),
        .pau_job_o              (pau_job),
        .pau_done_i             (pau_done),
        .pau_err_i              (4'h0), // See documentation header note 1

        // Memory Control/Status
        .mem_zeroize_req_o      (mem_zeroize_req),
        .mem_zeroize_done_i     (mem_zeroize_done),
        .mem_fault_i            (mem_fault),
        .mem_fault_code_i       (mem_fault_code),

        // HSU Hash_EK Authorization
        .hsu_hash_ek_read_en_o  (hsu_hash_ek_read_en),
        .mem_phase_o            (mem_phase),
        .active_sec_lvl_o       (active_sec_lvl)
    );

    // =========================================================================
    // Hash Sampler Unit (HSU)
    // =========================================================================
    hash_sampler_unit #(
        .COEFF_W(COEFF_WIDTH),
        .NCOEFF(NCOEFF),
        .NUM_POLYS(NUM_POLYS),
        .SEED_W(SEED_W),
        .SEED_BEATS(SEED_BEATS)
    ) u_hsu (
        .clk                    (clk),
        .rst                    (rst),

        .start_i                (hsu_start),
        .hsu_mode_i             (hsu_mode),
        .xof_len_i              (hsu_xof_len),
        .is_eta3_i              (hsu_is_eta3),

        .poly_id_i              (hsu_poly_id),
        .seed_id_i              (hsu_seed_id),
        .row_i                  (hsu_row),
        .col_i                  (hsu_col),
        .cbd_n_i                (hsu_cbd_n),

        .input_sel_i            (hsu_input_sel),
        .absorb_poly_i          (hsu_absorb_poly),
        .absorb_last_i          (hsu_absorb_last),

        // Poly Mem Writer
        .hsu_req_o              (hsu_mem_req),
        .hsu_rd_en_o            (hsu_mem_rd_en),
        .hsu_wr_poly_id_o       (hsu_mem_wr_poly_id),
        .hsu_wr_en_o            (hsu_mem_wr_en),
        .hsu_wr_idx_o           (hsu_mem_wr_idx),
        .hsu_wr_data_o          (hsu_mem_wr_data),
        .hsu_stall_i            (hsu_mem_stall),
        .hsu_done_o             (hsu_done),

        // Poly Mem Reader
        .hsu_rd_poly_id_o       (hsu_mem_rd_poly_id),
        .hsu_rd_idx_o           (hsu_mem_rd_idx),
        .hsu_rd_lane_valid_o    (hsu_mem_rd_lane_valid),
        .hsu_rd_lane_valid_i    (hsu_mem_rd_lane_valid_out),
        .hsu_rd_data_i          (hsu_mem_rd_data),
        .hsu_rd_valid_i         (hsu_mem_rd_valid),
        .hsu_rd_poly_id_i       (hsu_mem_rd_poly_id_out),
        .hsu_rd_idx_i           (hsu_mem_rd_idx_out),

        // Seed Mem
        .hsu_seed_req_o         (hsu_seed_req),
        .hsu_seed_we_o          (hsu_seed_we),
        .hsu_seed_id_o          (hsu_mem_seed_id),
        .hsu_seed_idx_o         (hsu_seed_idx),
        .hsu_seed_wdata_o       (hsu_seed_wdata),
        .hsu_seed_ready_i       (hsu_seed_ready),
        .hsu_seed_rvalid_i      (hsu_seed_rvalid),
        .hsu_seed_rdata_i       (hsu_seed_rdata),

        // Hash Snoop Input (from Transcoder)
        .axis_t_data_i          (hash_snoop_data),
        .axis_t_valid_i         (hash_snoop_valid),
        .axis_t_last_i          (hash_snoop_last),
        .axis_t_keep_i          (hash_snoop_keep),
        .axis_t_ready_o         (hash_snoop_ready),

        .packer_done_o          (hsu_packer_done)
    );

    // Mapping logic for PAU opcodes
    pe_mode_e pau_op_mapped;
    always_comb begin
        pau_op_mapped      = PE_MODE_IDLE;
        pau_poly_id        = '0;
        pau_cwm_num_terms  = '0;

        unique case (pau_job.opcode)
            PAU_JOB_NTT_IN_PLACE: begin
                pau_op_mapped     = PE_MODE_NTT;
                pau_poly_id       = pau_job.primary_poly_id;
                pau_cwm_num_terms = pau_job.k_active;
            end

            PAU_JOB_KEYGEN_ROWMAC: begin
                pau_op_mapped     = PE_MODE_CWM;
                pau_poly_id       = pau_job.row_idx;
                pau_cwm_num_terms = pau_job.k_active;
            end

            default: begin
                pau_op_mapped      = PE_MODE_IDLE;
                pau_poly_id        = '0;
                pau_cwm_num_terms  = '0;
            end
        endcase
    end

    // =========================================================================
    // Polynomial Arithmetic Unit (PAU)
    // =========================================================================
    poly_arith_unit #(
        .NUM_POLYS(NUM_POLYS)
    ) u_pau (
        .clk                    (clk),
        .rst                    (rst),

        .start_i                (pau_start),
        .op_type_i              (pau_op_mapped),
        .poly_id_i              (pau_poly_id),
        .cwm_num_terms_i        (pau_cwm_num_terms),
        .done_o                 (pau_done),

        // Primary Poly Mem Port
        .pau_req_o              (pau_mem_req),
        .pau_rd_en_o            (pau_mem_rd_en),
        .pau_rd_poly_id_o       (pau_mem_rd_poly_id),
        .pau_rd_idx_o           (pau_mem_rd_idx),
        .pau_rd_lane_valid_o    (pau_mem_rd_lane_valid),
        .pau_wr_en_o            (pau_mem_wr_en),
        .pau_wr_poly_id_o       (pau_mem_wr_poly_id),
        .pau_wr_idx_o           (pau_mem_wr_idx),
        .pau_wr_data_o          (pau_mem_wr_data_pau),
        .pau_rd_valid_i         (pau_mem_rd_valid),
        .pau_rd_poly_id_i       (pau_mem_rd_poly_id_out),
        .pau_rd_idx_i           (pau_mem_rd_idx_out),
        .pau_rd_lane_valid_i    (pau_mem_rd_lane_valid_out),
        .pau_rd_data_i          (pau_mem_rd_data_pau),
        .pau_stall_i            (pau_mem_stall),

        // Auxiliary Poly Mem Port
        .pau_aux_req_o          (pau_aux_req),
        .pau_aux_rd_en_o        (pau_aux_rd_en),
        .pau_aux_rd_poly_id_o   (pau_aux_rd_poly_id),
        .pau_aux_rd_idx_o       (pau_aux_rd_idx),
        .pau_aux_rd_lane_valid_o(pau_aux_rd_lane_valid),
        .pau_aux_wr_en_o        (pau_aux_wr_en),
        .pau_aux_wr_poly_id_o   (pau_aux_wr_poly_id),
        .pau_aux_wr_idx_o       (pau_aux_wr_idx),
        .pau_aux_wr_data_o      (pau_aux_wr_data_pau),
        .pau_aux_rd_valid_i     (pau_aux_rd_valid),
        .pau_aux_rd_poly_id_i   (pau_aux_rd_poly_id_out),
        .pau_aux_rd_idx_i       (pau_aux_rd_idx_out),
        .pau_aux_rd_lane_valid_i(pau_aux_rd_lane_valid_out),
        .pau_aux_rd_data_i      (pau_aux_rd_data_pau)
    );

    // =========================================================================
    // Memory Subsystem
    // =========================================================================
    poly_mem_subsystem #(
        .NUM_POLYS  (NUM_POLYS),
        .NCOEFF     (NCOEFF),
        .W          (16),
        .COEFF_W    (COEFF_WIDTH),
        .SEED_DEPTH (SEED_DEPTH),
        .SEED_W     (SEED_W)
    ) u_mem (
        .clk                    (clk),
        .rst                    (rst),

        // Security Wipe
        .wipe_i                 (mem_zeroize_req),
        .wipe_busy_o            (),
        .wipe_done_o            (mem_zeroize_done),
        .mem_fault_o            (mem_fault),
        .mem_fault_code_o       (mem_fault_code),

        // PAU Primary
        .pau_req                (pau_mem_req),
        .pau_rd_en              (pau_mem_rd_en),
        .pau_rd_poly_id         (pau_mem_rd_poly_id),
        .pau_rd_idx             (pau_mem_rd_idx),
        .pau_rd_lane_valid      (pau_mem_rd_lane_valid),
        .pau_wr_en              (pau_mem_wr_en),
        .pau_wr_poly_id         (pau_mem_wr_poly_id),
        .pau_wr_idx             (pau_mem_wr_idx),
        .pau_wr_data            (pau_mem_wr_data),
        .pau_rd_valid           (pau_mem_rd_valid),
        .pau_rd_poly_id_o       (pau_mem_rd_poly_id_out),
        .pau_rd_idx_o           (pau_mem_rd_idx_out),
        .pau_rd_lane_valid_o    (pau_mem_rd_lane_valid_out),
        .pau_rd_data            (pau_mem_rd_data),
        .pau_stall              (pau_mem_stall),

        // PAU Auxiliary
        .pau_aux_req            (pau_aux_req),
        .pau_aux_rd_en          (pau_aux_rd_en),
        .pau_aux_rd_poly_id     (pau_aux_rd_poly_id),
        .pau_aux_rd_idx         (pau_aux_rd_idx),
        .pau_aux_rd_lane_valid  (pau_aux_rd_lane_valid),
        .pau_aux_wr_en          (pau_aux_wr_en),
        .pau_aux_wr_poly_id     (pau_aux_wr_poly_id),
        .pau_aux_wr_idx         (pau_aux_wr_idx),
        .pau_aux_wr_data        (pau_aux_wr_data),
        .pau_aux_rd_valid       (pau_aux_rd_valid),
        .pau_aux_rd_poly_id_o   (pau_aux_rd_poly_id_out),
        .pau_aux_rd_idx_o       (pau_aux_rd_idx_out),
        .pau_aux_rd_lane_valid_o(pau_aux_rd_lane_valid_out),
        .pau_aux_rd_data        (pau_aux_rd_data),

        // HSU Constraints
        .hsu_hash_ek_read_en    (hsu_hash_ek_read_en),

        // HSU Poly
        .hsu_req                (hsu_mem_req),
        .hsu_rd_en              (hsu_mem_rd_en),
        .hsu_rd_poly_id         (hsu_mem_rd_poly_id),
        .hsu_rd_idx             (hsu_mem_rd_idx),
        .hsu_rd_lane_valid      (hsu_mem_rd_lane_valid),
        .hsu_wr_en              (hsu_mem_wr_en),
        .hsu_wr_poly_id         (hsu_mem_wr_poly_id),
        .hsu_wr_idx             (hsu_mem_wr_idx),
        .hsu_wr_data            (hsu_mem_wr_data),
        .hsu_rd_valid           (hsu_mem_rd_valid),
        .hsu_rd_poly_id_o       (hsu_mem_rd_poly_id_out),
        .hsu_rd_idx_o           (hsu_mem_rd_idx_out),
        .hsu_rd_lane_valid_o    (hsu_mem_rd_lane_valid_out),
        .hsu_rd_data            (hsu_mem_rd_data),
        .hsu_stall              (hsu_mem_stall),

        // Transcoder Poly
        .tr_req                 (tr_mem_req),
        .tr_rd_en               (tr_mem_rd_en),
        .tr_rd_poly_id          (tr_mem_rd_poly_id),
        .tr_rd_idx              (tr_mem_rd_idx),
        .tr_rd_lane_valid       (tr_mem_rd_lane_valid),
        .tr_wr_en               (tr_mem_wr_en),
        .tr_wr_poly_id          (tr_mem_wr_poly_id),
        .tr_wr_idx              (tr_mem_wr_idx),
        .tr_wr_data             (tr_mem_wr_data),
        .tr_rd_valid            (tr_mem_rd_valid),
        .tr_rd_poly_id_o        (tr_mem_rd_poly_id_out),
        .tr_rd_idx_o            (tr_mem_rd_idx_out),
        .tr_rd_lane_valid_o     (tr_mem_rd_lane_valid_out),
        .tr_rd_data             (tr_mem_rd_data),
        .tr_stall               (tr_mem_stall),

        // HSU Seed
        .hsu_seed_req           (hsu_seed_req),
        .hsu_seed_we            (hsu_seed_we),
        .hsu_seed_id            (hsu_mem_seed_id),
        .hsu_seed_idx           (hsu_seed_idx),
        .hsu_seed_wdata         (hsu_seed_wdata),
        .hsu_seed_ready         (hsu_seed_ready),
        .hsu_seed_rvalid        (hsu_seed_rvalid),
        .hsu_seed_rdata         (hsu_seed_rdata),

        // Transcoder Seed
        .tr_seed_req            (tr_seed_req),
        .tr_seed_we             (tr_seed_we),
        .tr_seed_id             (tr_seed_id),
        .tr_seed_idx            (tr_seed_idx),
        .tr_seed_wdata          (tr_seed_wdata),
        .tr_seed_ready          (tr_seed_ready),
        .tr_seed_rvalid         (tr_seed_rvalid),
        .tr_seed_rdata          (tr_seed_rdata)
    );

    // =========================================================================
    // Transcoder Unit
    // =========================================================================
    transcoder_unit u_tr (
        .clk                    (clk),
        .rst                    (rst),

        // Control
        .ctrl_start             (tr_start),
        .ctrl_done              (tr_done),
        .ctrl_sec_level         (active_sec_lvl),
        .ctrl_opcode            (tr_opcode),

        // Poly Mem
        .poly_req_o             (tr_mem_req),
        .poly_stall_i           (tr_mem_stall),
        .poly_rd_en_o           (tr_mem_rd_en),
        .poly_rd_poly_id_o      (tr_mem_rd_poly_id),
        .poly_rd_idx_o          (tr_mem_rd_idx),
        .poly_rd_lane_valid_o   (tr_mem_rd_lane_valid),
        .poly_wr_en_o           (tr_mem_wr_en),
        .poly_wr_poly_id_o      (tr_mem_wr_poly_id),
        .poly_wr_idx_o          (tr_mem_wr_idx),
        .poly_wr_data_o         (tr_mem_wr_data),
        .poly_rd_valid_i        (tr_mem_rd_valid),
        .poly_rd_poly_id_i      (tr_mem_rd_poly_id_out),
        .poly_rd_idx_i          (tr_mem_rd_idx_out),
        .poly_rd_lane_valid_i   (tr_mem_rd_lane_valid_out),
        .poly_rd_data_i         (tr_mem_rd_data),

        // Seed Mem
        .seed_req_o             (tr_seed_req),
        .seed_we_o              (tr_seed_we),
        .seed_id_o              (tr_seed_id),
        .seed_idx_o             (tr_seed_idx),
        .seed_wdata_o           (tr_seed_wdata),
        .seed_ready_i           (tr_seed_ready),
        .seed_rvalid_i          (tr_seed_rvalid),
        .seed_rdata_i           (tr_seed_rdata),

        // Hash Snoop
        .hash_snoop_data_o      (hash_snoop_data),
        .hash_snoop_keep_o      (hash_snoop_keep),
        .hash_snoop_valid_o     (hash_snoop_valid),
        .hash_snoop_ready_i     (hash_snoop_ready),
        .hash_snoop_last_o      (hash_snoop_last),

        // External AXI-Stream
        .s_axis_tdata           (s_axis_tdata_i),
        .s_axis_tvalid          (s_axis_tvalid_i),
        .s_axis_tready          (s_axis_tready_o),
        .s_axis_tlast           (s_axis_tlast_i),
        .m_axis_tdata           (m_axis_tdata_o),
        .m_axis_tvalid          (m_axis_tvalid_o),
        .m_axis_tready          (m_axis_tready_i),
        .m_axis_tkeep           (m_axis_tkeep_o),
        .m_axis_tlast           (m_axis_tlast_o)
    );

endmodule
