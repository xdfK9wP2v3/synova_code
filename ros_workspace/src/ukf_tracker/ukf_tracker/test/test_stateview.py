import unittest
from ukf_filter.State import *


class TestKineticView(unittest.TestCase):
    def test_init(self):
        # initial safe check
        with self.assertRaises(AssertionError):
            KineticView('QPVV')
        with self.assertRaises(AssertionError):
            KineticView('QPMV')

        view = KineticView('QOPV')
        self.assertEqual(view._source_kinetic_seq, 'QOPV')
        self.assertEqual(view._view_kinetic_seq, 'QOPV')

        with self.assertRaises(AssertionError):
            KineticView('QOPV', 'QPVV')
        with self.assertRaises(AssertionError):
            KineticView('QOPV', 'QPMV')
        with self.assertRaises(AssertionError):
            KineticView('QOPV', 'QOPVA')

        view = KineticView('QOPV', 'QP')
        self.assertEqual(view._source_kinetic_seq, 'QOPV')
        self.assertEqual(view._view_kinetic_seq, 'QP')

    def test_mapping(self):
        self.assertListEqual(KineticView('QOPV', 'QP').slice_mapping,
                             [(slice(0, 4), slice(0, 4)), (slice(7, 10), slice(4, 7))])

        self.assertListEqual(KineticView('QOPV', 'QOP').slice_mapping,
                             [(slice(0, 10), slice(0, 10))])

        self.assertListEqual(KineticView('QOPV', 'PQ').slice_mapping,
                             [(slice(7, 10), slice(0, 3)), (slice(0, 4), slice(3, 7))])

        self.assertListEqual(KineticView('QOPV', 'PVQ').slice_mapping,
                             [(slice(7, 13), slice(0, 6)), (slice(0, 4), slice(6, 10))])

    def test_getsub_view(self):
        view = KineticView('QOPV', 'QP')
        with self.assertRaises(AssertionError):
            _ = view['M']
        with self.assertRaises(AssertionError):
            _ = view['PP']
        with self.assertRaises(AssertionError):
            _ = view['V']

        subview = view['P']
        self.assertEqual(subview._source_kinetic_seq, 'QOPV')
        self.assertEqual(subview._view_kinetic_seq, 'P')

    def test_formate(self):
        self.assertEqual(KineticView.format_seq({'P', 'V', 'Q'}), 'QPV')
        self.assertEqual(KineticView.format_seq({'P', 'O', 'Q'}), 'QOP')
        self.assertEqual(KineticView.format_seq({'A', 'V', 'Q'}), 'QVA')


class TestMeanView(unittest.TestCase):
    def setUp(self):
        self.batch = 8
        self.m = np.arange(self.batch * 13).reshape(-1, 13)
        self.view1 = MeanView(self.m, KineticView('QOPV', 'QP'))
        self.view2 = MeanView(self.m, KineticView('QOPV', 'QO'))
        self.view3 = self.view1('P')
        self.view4 = MeanView(self.m, KineticView('QOPV', 'VP'))

    def test_subview(self):
        with self.assertRaises(AssertionError):
            _ = self.view1('M')
        with self.assertRaises(AssertionError):
            _ = self.view1('PP')
        with self.assertRaises(AssertionError):
            _ = self.view1('V')

        subview = self.view1('P')
        self.assertEqual(subview._view._source_kinetic_seq, 'QOPV')
        self.assertEqual(subview._view._view_kinetic_seq, 'P')

    def test_init(self):
        with self.assertRaises(AssertionError):
            MeanView(self.m, KineticView('QPV'))

        self.m[0, 4] = 1000
        self.assertEqual(self.view1._arr[0, 4], 1000)

        self.view1._arr[0, 7] = 500
        self.assertEqual(self.m[0, 7], 500)

    def test_getter(self):
        self.assertTrue(np.all(self.view1.value[..., 0:4] == self.m[..., 0:4]))
        self.assertTrue(np.all(self.view1.value[..., 4:7] == self.m[..., 7:10]))

        self.assertTrue(np.all(self.view2.value == self.m[..., 0:7]))

        self.assertTrue(np.all(self.view3.value == self.m[..., 7:10]))

        self.assertTrue(np.all(self.view4.value[..., 0:3] == self.m[..., 10:13]))
        self.assertTrue(np.all(self.view4.value[..., 3:6] == self.m[..., 7:10]))

        self.assertTrue(np.all(self.view1[...][..., 0:4] == self.m[..., 0:4]))
        self.assertTrue(np.all(self.view1[...][..., 4:7] == self.m[..., 7:10]))
        self.assertTrue(np.all(self.view1[3, ...][..., 0:4] == self.m[3, 0:4]))
        self.assertTrue(np.all(self.view1[3, ...][..., 4:7] == self.m[3, 7:10]))
        self.assertTrue(np.all(self.view1['Q'] == self.m[..., 0:4]))
        self.assertTrue(np.all(self.view1[3, 'Q'] == self.m[3, 0:4]))

    def test_setter(self):
        with self.assertRaises(AssertionError):
            self.view1.value = np.empty((self.batch, 8))
        with self.assertRaises(AssertionError):
            self.view1.value = np.empty((self.batch, 6))
        with self.assertRaises(AssertionError):
            self.view1.value = np.empty((self.batch + 4, 7))

        self.view1.value = np.ones((self.batch, 7)) * -1
        self.assertTrue(np.all(self.m[..., 0:4] == -1))
        self.assertTrue(np.all(self.m[..., 4:7] >= 0))
        self.assertTrue(np.all(self.m[..., 7:10] == -1))
        self.assertTrue(np.all(self.m[..., 10:13] >= 0))

        self.view2.value = np.ones((self.batch, 7)) * -2
        self.assertTrue(np.all(self.m[..., 0:4] == -2))
        self.assertTrue(np.all(self.m[..., 4:7] == -2))
        self.assertTrue(np.all(self.m[..., 7:10] == -1))
        self.assertTrue(np.all(self.m[..., 10:13] >= 0))

        self.view3.value = np.ones((self.batch, 3)) * -3
        self.assertTrue(np.all(self.m[..., 0:4] == -2))
        self.assertTrue(np.all(self.m[..., 4:7] == -2))
        self.assertTrue(np.all(self.m[..., 7:10] == -3))
        self.assertTrue(np.all(self.m[..., 10:13] >= 0))

        self.view4.value = np.ones((self.batch, 6)) * -4
        self.assertTrue(np.all(self.m[..., 0:4] == -2))
        self.assertTrue(np.all(self.m[..., 4:7] == -2))
        self.assertTrue(np.all(self.m[..., 7:10] == -4))
        self.assertTrue(np.all(self.m[..., 10:13] == -4))


class TestCovarianceView(unittest.TestCase):
    def setUp(self):
        self.batch = 8
        self.P = np.arange(self.batch * 13 * 13).reshape(-1, 13, 13)
        self.view1 = CovarianceView(self.P, KineticView('QOPV', 'QP'))
        self.view2 = CovarianceView(self.P, KineticView('QOPV', 'QO'))
        self.view3 = self.view1('P')
        self.view4 = CovarianceView(self.P, KineticView('QOPV', 'VP'))

    def test_subview(self):
        with self.assertRaises(AssertionError):
            _ = self.view1('M')
        with self.assertRaises(AssertionError):
            _ = self.view1('PP')
        with self.assertRaises(AssertionError):
            _ = self.view1('V')

        subview = self.view1('P')
        self.assertEqual(subview._view._source_kinetic_seq, 'QOPV')
        self.assertEqual(subview._view._view_kinetic_seq, 'P')

    def test_reference(self):
        self.P[0, 4, 1] = 1000
        self.assertEqual(self.view1._arr[0, 4, 1], 1000)

        self.view1._arr[0, 7, 2] = 500
        self.assertEqual(self.P[0, 7, 2], 500)

    def test_getter(self):
        self.assertTrue(np.all(self.view1.value[..., 0:4, 0:4] == self.P[..., 0:4, 0:4]))
        self.assertTrue(np.all(self.view1.value[..., 4:7, 0:4] == self.P[..., 7:10, 0:4]))
        self.assertTrue(np.all(self.view1.value[..., 0:4, 4:7] == self.P[..., 0:4, 7:10]))
        self.assertTrue(np.all(self.view1.value[..., 4:7, 4:7] == self.P[..., 7:10, 7:10]))

        self.assertTrue(np.all(self.view2.value == self.P[..., 0:7, 0:7]))

        self.assertTrue(np.all(self.view3.value == self.P[..., 7:10, 7:10]))

        self.assertTrue(np.all(self.view4.value[..., 0:3, 0:3] == self.P[..., 10:13, 10:13]))
        self.assertTrue(np.all(self.view4.value[..., 3:6, 0:3] == self.P[..., 7:10, 10:13]))
        self.assertTrue(np.all(self.view4.value[..., 0:3, 3:6] == self.P[..., 10:13, 7:10]))
        self.assertTrue(np.all(self.view4.value[..., 3:6, 3:6] == self.P[..., 7:10, 7:10]))

        self.assertTrue(np.all(self.view1[...][..., 0:4, 0:4] == self.P[..., 0:4, 0:4]))
        self.assertTrue(np.all(self.view1[3, ...][..., 0:4, 0:4] == self.P[3, 0:4, 0:4]))
        self.assertTrue(np.all(self.view1['Q'] == self.P[..., 0:4, 0:4]))
        self.assertTrue(np.all(self.view1[3, 'Q'] == self.P[3, 0:4, 0:4]))

    def test_setter(self):
        with self.assertRaises(AssertionError):
            self.view1.value = np.empty((self.batch, 7, 8))
        with self.assertRaises(AssertionError):
            self.view1.value = np.empty((self.batch, 7, 6))
        with self.assertRaises(AssertionError):
            self.view1.value = np.empty((self.batch + 4, 7, 7))

        self.view1.value = np.ones((self.batch, 7, 7)) * -1
        self.assertTrue(np.all(self.P[..., 4:7, :] >= 0))
        self.assertTrue(np.all(self.P[..., :, 4:7] >= 0))
        self.assertTrue(np.all(self.P[..., 10:13, :] >= 0))
        self.assertTrue(np.all(self.P[..., :, 10:13] >= 0))
        self.assertTrue(np.all(self.P[..., 0:4, 0:4] == -1))
        self.assertTrue(np.all(self.P[..., 7:10, 0:4] == -1))
        self.assertTrue(np.all(self.P[..., 0:4, 7:10] == -1))
        self.assertTrue(np.all(self.P[..., 7:10, 7:10] == -1))

        self.view2.value = np.ones((self.batch, 7, 7)) * -2
        self.assertTrue(np.all(self.P[..., 0:7, 0:7] == -2))
        self.assertTrue(np.all(self.P[..., :, 10:13] >= 0))
        self.assertTrue(np.all(self.P[..., 10:13, :] >= 0))

        self.view3.value = np.ones((self.batch, 3, 3)) * -3
        self.assertTrue(np.all(self.P[..., 7:10, 7:10] == -3))

        self.view4.value = np.ones((self.batch, 6, 6)) * -4
        self.assertTrue(np.all(self.P[..., 10:13, 10:13] == -4))


if __name__ == '__main__':
    unittest.main()
